"""Independent PyQt review process; analysis runs in the isolated engine Python."""
import json,shutil,sys
from datetime import datetime
from pathlib import Path
try:
    import PyQt6
except ModuleNotFoundError:
    # Reuse this interpreter's existing Qt installation only in the UI process.
    # Engine subprocesses reset PYTHONPATH and retain the isolated environment.
    sys.path.append(str(Path(sys.base_prefix)/'Lib/site-packages'))
from PyQt6.QtCore import Qt,QUrl,QProcess,QProcessEnvironment,QTimer,QRectF
from PyQt6.QtGui import QFont,QFontDatabase,QPainter,QImage
from PyQt6.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout,QPushButton,QLineEdit,QFileDialog,QLabel,QPlainTextEdit,QSlider,QMessageBox,QSplitter,QComboBox,QDialog,QFormLayout,QDoubleSpinBox,QDialogButtonBox,QCheckBox
from PyQt6.QtMultimedia import QMediaPlayer,QAudioOutput,QVideoSink

ROOT=Path(__file__).resolve().parents[2]


def model_assets_root():
    candidates=[ROOT.parent/'fsd-sedan-3d/models', ROOT.parent.parent/'fsd-sedan-3d/models']
    return next((p for p in candidates if (p/'tesla_fsd_sedan.glb').is_file()), candidates[0])


class VideoCanvas(QWidget):
    def __init__(self):
        super().__init__();self.sink=QVideoSink(self);self.image=QImage();self.sink.videoFrameChanged.connect(self.receive)

    def videoSink(self):return self.sink

    def receive(self,frame):
        if frame.isValid():self.image=frame.toImage();self.update()

    def paintEvent(self,event):
        painter=QPainter(self);painter.fillRect(self.rect(),Qt.GlobalColor.black)
        if self.image.isNull():return
        size=self.image.size().scaled(self.size(),Qt.AspectRatioMode.KeepAspectRatio)
        target=QRectF((self.width()-size.width())/2,(self.height()-size.height())/2,size.width(),size.height())
        painter.drawImage(target,self.image)


def result_directory(path):
    p=Path(path).resolve()
    if p.is_file():p=p.parent
    if (p/'pipeline.json').exists():p=p/'scene'
    from .artifacts import verify
    m=verify(p)
    if not (p/'surrounding-world.mp4').is_file():raise ValueError('Choose a v2 scene result with an exported video')
    return p,m


def recent_results(root,limit=8):
    """List published full exports; loading still verifies their payload hashes."""
    candidates=[]
    for pattern in ('*/artifact.json','*/*/artifact.json','*/*/*/artifact.json'):
        for path in Path(root).glob(pattern):
            try:
                manifest=json.loads(path.read_text(encoding='utf-8'))
                if manifest.get('stage') not in ('surrounding_world_prototype','scene_replay') or manifest.get('status')!='complete':continue
                if not (path.parent/'surrounding-world.mp4').is_file():continue
                candidates.append((manifest.get('created_utc',''),path.parent))
            except (OSError,ValueError):continue
    result=[]
    for stamp,path in sorted(candidates,key=lambda r:(r[0],str(r[1])),reverse=True):
        try:
            report=json.loads((path/'report.json').read_text(encoding='utf-8'))
            if report.get('render_is_preview') or not report.get('export_full_source'):continue
            result.append((path,float(report['duration_s'])))
            if len(result)>=limit:break
        except (OSError,ValueError,KeyError,TypeError):continue
    return result


class ReviewWindow(QWidget):
    def __init__(self):
        super().__init__();self.setWindowTitle('CT Scene v2 · 주행 재현 검토');self.resize(1160,1000)
        font_path=Path('C:/Windows/Fonts/malgun.ttf')
        if font_path.exists():QFontDatabase.addApplicationFont(str(font_path))
        self.setFont(QFont('Malgun Gothic',10))
        self.process=None;self.result=None;self.states=[];self.manifest=None;self.pending_result=None
        layout=QVBoxLayout(self);source_row=QHBoxLayout();self.source=QLineEdit();self.source.setPlaceholderText('원본 *-front.mp4 선택 — 같은 시각 4채널 전체 구간')
        choose=QPushButton('원본 선택');choose.clicked.connect(self.choose_source);source_row.addWidget(self.source);source_row.addWidget(choose);layout.addLayout(source_row)
        output_row=QHBoxLayout();self.output=QLineEdit(str(ROOT/'runs'/('lab-v2-'+datetime.now().strftime('%Y%m%d-%H%M%S'))))
        choose_out=QPushButton('작업 위치');choose_out.clicked.connect(self.choose_output);output_row.addWidget(self.output);output_row.addWidget(choose_out);layout.addLayout(output_row)
        map_row=QHBoxLayout();self.map=QLineEdit();self.map.setPlaceholderText('OSM 캐시 선택 사항 — 도로 윤곽 참고, GPS 차로 배정 없음')
        choose_map=QPushButton('OSM 캐시');choose_map.clicked.connect(self.choose_map);map_row.addWidget(self.map);map_row.addWidget(choose_map);layout.addLayout(map_row)
        self.auto_map=QCheckBox('지도 미지정 시 해당 영상의 로컬 OSM 캐시 자동 선택 · 네트워크 조회 없음');self.auto_map.setChecked(True);layout.addWidget(self.auto_map)
        buttons=QHBoxLayout();self.start=QPushButton('전체 분석 / 이어하기');self.start.clicked.connect(self.analyze);self.cancel=QPushButton('분석 취소');self.cancel.clicked.connect(self.stop);self.cancel.setEnabled(False)
        open_button=QPushButton('v2 결과 열기');open_button.clicked.connect(self.choose_result);self.export=QPushButton('MP4 내보내기');self.export.clicked.connect(self.save_video);self.export.setEnabled(False)
        self.backend=QComboBox();self.backend.addItem('GPU · DirectML','directml');self.backend.addItem('CPU','cpu')
        for b in (self.start,self.cancel,open_button,self.export):buttons.addWidget(b)
        buttons.addWidget(self.backend)
        self.manual=QPushButton('차량 수동 교정본');self.manual.setEnabled(False);self.manual.clicked.connect(self.manual_copy);buttons.addWidget(self.manual)
        layout.addLayout(buttons);self.status=QLabel('실험용 v2 · 추정 위치 · 후방은 식별 참고 · 원본/기존 결과 보존');self.status.setWordWrap(True);layout.addWidget(self.status)
        recent_row=QHBoxLayout();recent_row.addWidget(QLabel('최근 완료 결과'))
        self.recent=QComboBox();recent_row.addWidget(self.recent,1)
        refresh=QPushButton('새로고침');refresh.clicked.connect(self.refresh_recent);recent_row.addWidget(refresh)
        self.open_recent=QPushButton('선택 결과 열기');self.open_recent.clicked.connect(self.load_recent);recent_row.addWidget(self.open_recent)
        layout.addLayout(recent_row);self.refresh_recent()
        self.replay=QPushButton('화면 설정 / 다시 렌더 · 추론 재사용');self.replay.setEnabled(False)
        self.replay.clicked.connect(self.replay_copy);layout.addWidget(self.replay)
        splitter=QSplitter(Qt.Orientation.Vertical);self.video=VideoCanvas();self.video.setMinimumHeight(420);splitter.addWidget(self.video)
        self.log=QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumBlockCount(1500);splitter.addWidget(self.log);splitter.setSizes([740,100]);layout.addWidget(splitter,1)
        playback=QHBoxLayout();self.play=QPushButton('재생 / 일시정지');self.play.clicked.connect(self.toggle);self.slider=QSlider(Qt.Orientation.Horizontal);self.slider.sliderMoved.connect(self.seek);self.clock=QLabel('00:00 / 00:00')
        playback.addWidget(self.play);playback.addWidget(self.slider,1);playback.addWidget(self.clock);layout.addLayout(playback)
        self.player=QMediaPlayer(self);self.audio=QAudioOutput(self);self.audio.setVolume(0);self.player.setAudioOutput(self.audio);self.player.setVideoSink(self.video.videoSink())
        self.player.positionChanged.connect(self.position);self.player.durationChanged.connect(lambda d:self.slider.setRange(0,d));self.player.errorOccurred.connect(lambda error,message:self.log.appendPlainText('재생: '+message))
        self.setStyleSheet('QWidget{background:#202329;color:#e3e6eb;} QLineEdit,QPlainTextEdit{background:#14171c;border:1px solid #454a53;padding:5px;} QPushButton{padding:7px;border:1px solid #555b66;border-radius:4px;} QPushButton:disabled{color:#737782;} QSlider::groove:horizontal{height:5px;background:#4b515b;} QSlider::handle:horizontal{background:#b6c2d0;width:12px;margin:-4px 0;}')

    def choose_source(self):
        path,_=QFileDialog.getOpenFileName(self,'TeslaCam 전방 원본',str(ROOT),'TeslaCam (*-front.mp4)')
        if path:self.source.setText(path)

    def refresh_recent(self):
        self.recent.clear()
        for path,duration in recent_results(ROOT/'runs'):
            self.recent.addItem(f'{path.relative_to(ROOT / "runs")} · {duration:.1f}초',str(path))
        self.open_recent.setEnabled(self.recent.count()>0)

    def load_recent(self):
        path=self.recent.currentData()
        if path:self.load(path)

    def choose_output(self):
        path=QFileDialog.getExistingDirectory(self,'새 작업 폴더 또는 이어할 작업 폴더',str(ROOT/'runs'))
        if path:self.output.setText(path)

    def choose_map(self):
        path,_=QFileDialog.getOpenFileName(self,'지역이 맞는 OSM 캐시',str(ROOT/'map-cache'),'OSM (*.osm *.xml *.json)')
        if path:self.map.setText(path)

    def choose_result(self):
        path=QFileDialog.getExistingDirectory(self,'v2 scene 결과 또는 pipeline 폴더',str(ROOT/'runs'))
        if path:self.load(path)

    def load(self,path):
        try:
            result,manifest=result_directory(path);report=json.loads((result/'report.json').read_text(encoding='utf-8'))
            self.states=[json.loads(l) for l in (result/'render-states.jsonl').read_text(encoding='utf-8').splitlines()]
        except (ValueError,OSError,KeyError) as error:self.log.appendPlainText('결과 열기 실패: '+str(error));return False
        self.result=result;self.manifest=manifest;self.player.setSource(QUrl.fromLocalFile(str(result/'surrounding-world.mp4')));self.export.setEnabled(True)
        self.manual.setEnabled(manifest['stage']=='surrounding_world_prototype')
        self.replay_compatible=bool(self.states) and all('ego_position_W' in s and 'ego_yaw_rad' in s and all('model' in a for a in s['actors']) for s in self.states)
        self.replay.setEnabled(self.process is None and self.replay_compatible and manifest['stage'] in ('surrounding_world_prototype','scene_replay'))
        self.replay.setToolTip('차량·도로 좌표를 유지하고 화면만 다시 렌더합니다.' if self.replay_compatible else '이전 결과에는 필요한 장면 상태가 없습니다. 최신 분석 결과에서 사용할 수 있습니다.')
        self.status.setText(f'v2 결과 · {report["duration_s"]:.1f}초 · {report["global_actors"]}개 전역 ID · 실측 정확도 미평가 · 전체 품질은 원본 대조 필요')
        if report.get('manual_corrections'):self.status.setText('수동 교정본 · 원본 자동 결과 보존 · 자동 성능 평가에서 제외')
        self.log.appendPlainText('결과: '+str(result));return True

    def analyze(self):
        if self.process is not None:return
        from .ingest import discover
        try:
            cameras=discover(self.source.text())
            if not {'front','back','left_repeater','right_repeater'}<=cameras.keys():raise ValueError('같은 시각의 원본 4채널이 필요합니다.')
            if Path(self.output.text()).resolve()==ROOT:raise ValueError('runs 아래 별도의 작업 폴더를 선택하세요.')
        except (ValueError,OSError) as error:self.log.appendPlainText(str(error));return
        python=ROOT/'.venv/Scripts/python.exe';assets=model_assets_root()
        args=['-m','ct_scene','analyze','--front',self.source.text(),'--output',self.output.text(),'--detector',str(ROOT.parent/'scene-models/yolo11x.onnx'),'--road-model',str(ROOT.parent/'scene-models/model-review/yolopv2-roads.onnx'),'--lens-config',str(ROOT/'configs/multicam-optical-candidates.json'),'--prototype-config',str(ROOT/'configs/surrounding-prototype.json'),'--assets-root',str(assets)]
        map_path=self.map.text().strip()
        if not map_path and self.auto_map.isChecked():
            candidate=ROOT/'map-cache'/(Path(self.source.text()).name.removesuffix('-front.mp4')+'.osm')
            if candidate.is_file():map_path=str(candidate);self.log.appendPlainText('로컬 OSM 캐시 자동 선택: '+map_path)
        if map_path:args+=['--osm',map_path]
        args+=['--backend',self.backend.currentData()]
        self.process=QProcess(self);env=QProcessEnvironment.systemEnvironment();env.insert('PYTHONPATH',str(ROOT/'src'));env.insert('PYTHONUNBUFFERED','1');self.process.setProcessEnvironment(env);self.process.setWorkingDirectory(str(ROOT));self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_log);self.process.finished.connect(self.finished);self.process.errorOccurred.connect(self.process_error)
        self.pending_result=Path(self.output.text());self.manual.setEnabled(False);self.replay.setEnabled(False)
        self.start.setEnabled(False);self.cancel.setEnabled(True);self.status.setText('분석 중 · 취소 후 같은 작업 위치에서 완료 단계를 검증하고 이어할 수 있습니다.');self.process.start(str(python),args)

    def read_log(self):
        if not self.process:return
        for line in bytes(self.process.readAllStandardOutput()).decode('utf-8',errors='replace').splitlines():
            try:
                event=json.loads(line)
                if isinstance(event,dict) and event.get('integrity')=='FAIL':line=str(event.get('reason',line))
            except ValueError:pass
            self.log.appendPlainText(line)

    def process_error(self,error):
        if error==QProcess.ProcessError.FailedToStart:
            self.log.appendPlainText('분석 프로세스를 시작하지 못했습니다. 격리 Python 경로를 확인하세요.');self.finished(-1,None)

    def finished(self,code,status):
        self.read_log();process=self.process;self.process=None;self.start.setEnabled(True);self.cancel.setEnabled(False)
        if process:process.deleteLater()
        self.manual.setEnabled(bool(self.manifest and self.manifest['stage']=='surrounding_world_prototype'))
        self.replay.setEnabled(bool(self.manifest and getattr(self,'replay_compatible',False) and self.manifest['stage'] in ('surrounding_world_prototype','scene_replay')))
        if code==0:
            self.load(self.pending_result or self.output.text());self.refresh_recent()
        else:self.status.setText('작업 중단/실패 · 원본 결과 보존 · 로그 확인')
        self.pending_result=None

    def replay_copy(self):
        if self.process or not self.result or self.manifest['stage'] not in ('surrounding_world_prototype','scene_replay'):return
        self.player.pause();report=json.loads((self.result/'report.json').read_text(encoding='utf-8'))
        config=json.loads((ROOT/'configs/surrounding-prototype.json').read_text(encoding='utf-8'))
        style=report.get('replay_style',config['render_style']).copy()
        dialog=QDialog(self);dialog.setWindowTitle('화면 설정 · 차량·도로 좌표 유지');form=QFormLayout(dialog)
        form.addRow(QLabel('기존 장면으로 영상을 다시 만듭니다. 차량 추론과 위치 계산은 반복하지 않습니다.'))
        zoom=QDoubleSpinBox();zoom.setRange(.8,2.5);zoom.setSingleStep(.1);zoom.setDecimals(2);zoom.setValue(style.get('camera_zoom',1.3));form.addRow('화면 확대 배율',zoom)
        brightness=QDoubleSpinBox();brightness.setRange(1,25);brightness.setDecimals(1);brightness.setSingleStep(1)
        old_rgb=style.get('road_rgb',[.035,.04,.05]);old_level=sum(old_rgb)/3
        brightness.setValue(old_level*100);form.addRow('도로 밝기 (%)',brightness)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel);buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);form.addRow(buttons)
        if dialog.exec()!=QDialog.DialogCode.Accepted:return
        style['camera_zoom']=zoom.value();style['road_rgb']=[min(1,c*brightness.value()/100/max(old_level,1e-6)) for c in old_rgb]
        config['render_style']=style
        target=ROOT/'runs'/('scene-replay-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        config_path=target.with_suffix('.display.json');config_path.write_text(json.dumps(config,indent=2),encoding='utf-8')
        args=['-m','ct_scene','replay-scene','--scene',str(self.result),'--assets-root',str(model_assets_root()),'--prototype-config',str(config_path),'--output',str(target)]
        self.process=QProcess(self);env=QProcessEnvironment.systemEnvironment();env.insert('PYTHONPATH',str(ROOT/'src'));env.insert('PYTHONUNBUFFERED','1')
        self.process.setProcessEnvironment(env);self.process.setWorkingDirectory(str(ROOT));self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_log);self.process.finished.connect(self.finished);self.process.errorOccurred.connect(self.process_error)
        self.pending_result=target;self.start.setEnabled(False);self.manual.setEnabled(False);self.replay.setEnabled(False);self.cancel.setEnabled(True)
        self.status.setText('화면 다시 렌더 중 · 기존 추론·좌표 재사용 · 원본 결과 보존');self.process.start(str(ROOT/'.venv/Scripts/python.exe'),args)

    def manual_copy(self):
        if self.process or not self.result or self.manifest['stage']!='surrounding_world_prototype':return
        self.player.pause();t=self.player.position()/1000
        state=min(self.states,key=lambda s:abs(s['time_s']-t))
        if not state['actors']:self.log.appendPlainText('현재 시점에 위치가 있는 차량이 없습니다. 차량이 보이는 시점으로 이동하세요.');return
        dialog=QDialog(self);dialog.setWindowTitle('차량 수동 교정 · 별도 결과');form=QFormLayout(dialog)
        form.addRow(QLabel('현재 시점의 위치·방향을 교정합니다. 구간 양 끝에서 자동 위치로 부드럽게 돌아옵니다.'))
        actor=QComboBox();actor.addItems([a['id'] for a in state['actors']]);form.addRow('차량 ID',actor)
        def number(label,lo,hi,value):
            spin=QDoubleSpinBox();spin.setRange(lo,hi);spin.setDecimals(3);spin.setValue(value);form.addRow(label,spin);return spin
        begin=number('시작 (초)',self.states[0]['time_s'],self.states[-1]['time_s'],max(self.states[0]['time_s'],t-1))
        peak=number('교정 시점 (초)',self.states[0]['time_s'],self.states[-1]['time_s'],t)
        end=number('종료 (초)',self.states[0]['time_s'],self.states[-1]['time_s'],min(self.states[-1]['time_s'],t+1))
        dx=number('월드 X 이동 (m)',-1000,1000,0);dy=number('월드 Y 이동 (m)',-1000,1000,0);angle=number('방향 보정 (도)',-180,180,0)
        reason=QLineEdit();form.addRow('원본 대조 근거',reason)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel);buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);form.addRow(buttons)
        if dialog.exec()!=QDialog.DialogCode.Accepted:return
        layer=dict(schema='ct_scene.manual_corrections.v1',source_artifact_id=self.manifest['artifact_id'],edits=[dict(actor_id=actor.currentText(),reason=reason.text(),keyframes=[dict(time_s=begin.value(),offset_W_xy_m=[0.,0.],yaw_delta_deg=0.),dict(time_s=peak.value(),offset_W_xy_m=[dx.value(),dy.value()],yaw_delta_deg=angle.value()),dict(time_s=end.value(),offset_W_xy_m=[0.,0.],yaw_delta_deg=0.)])])
        try:
            from .manual_review import validate
            validate(layer,self.manifest,self.states)
            if dx.value()==dy.value()==angle.value()==0:raise ValueError('교정할 위치 또는 방향 값을 입력하세요.')
            wanted=next(i['artifact_id'] for i in self.manifest['inputs'] if i['role']=='frozen_ego_lead')
            # Resolve only a content-identified input. Never choose a nearby run by name.
            candidates=[self.result.parent/'base/artifact.json']
            candidates.extend((ROOT/'runs').rglob('artifact.json'));base=None
            for path in candidates:
                if not path.is_file():continue
                try:m=json.loads(path.read_text(encoding='utf-8'))
                except (OSError,ValueError):continue
                if m.get('artifact_id')==wanted:base=path.parent;break
            if base is None:raise ValueError('연결된 자동 분석 base를 찾을 수 없습니다. 원본 작업 폴더를 보존하세요.')
            stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f');target=ROOT/'runs'/('manual-review-'+stamp);layer_path=target.with_suffix('.corrections.json')
            layer_path.write_text(json.dumps(layer,ensure_ascii=False,indent=2),encoding='utf-8')
        except (ValueError,OSError,KeyError,StopIteration) as error:self.log.appendPlainText('교정 준비 실패: '+str(error));return
        args=['-m','ct_scene','manual-render','--scene',str(self.result),'--base',str(base),'--corrections',str(layer_path),'--assets-root',str(model_assets_root()),'--prototype-config',str(ROOT/'configs/surrounding-prototype.json'),'--output',str(target)]
        self.process=QProcess(self);env=QProcessEnvironment.systemEnvironment();env.insert('PYTHONPATH',str(ROOT/'src'));env.insert('PYTHONUNBUFFERED','1');self.process.setProcessEnvironment(env);self.process.setWorkingDirectory(str(ROOT));self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_log);self.process.finished.connect(self.finished);self.process.errorOccurred.connect(self.process_error)
        self.pending_result=target;self.start.setEnabled(False);self.manual.setEnabled(False);self.replay.setEnabled(False);self.cancel.setEnabled(True);self.status.setText('수동 교정본 생성 중 · 자동 결과 보존');self.process.start(str(ROOT/'.venv/Scripts/python.exe'),args)

    def stop(self):
        if not self.process:return
        process=self.process;process.terminate();QTimer.singleShot(3000,lambda:process.kill() if self.process is process and process.state()!=QProcess.ProcessState.NotRunning else None)

    def toggle(self):
        self.player.pause() if self.player.playbackState()==QMediaPlayer.PlaybackState.PlayingState else self.player.play()

    def seek(self,value):self.player.setPosition(value)

    def position(self,value):
        if not self.slider.isSliderDown():self.slider.setValue(value)
        fmt=lambda ms:f'{int(ms//60000):02d}:{int(ms//1000%60):02d}'
        self.clock.setText(fmt(value)+' / '+fmt(self.player.duration()))

    def save_video(self):
        if not self.result:return
        target,_=QFileDialog.getSaveFileName(self,'같은 검토 영상 MP4 저장',str(Path.home()/'Downloads/CT-Scene-v2.mp4'),'MP4 (*.mp4)')
        if not target:return
        try:
            path=Path(target)
            if path.resolve().is_relative_to(self.result.resolve()):raise ValueError('분석 결과 폴더 밖의 저장 위치를 선택하세요.')
            shutil.copy2(self.result/'surrounding-world.mp4',path)
            path.with_suffix('.export.json').write_text(json.dumps(dict(source_artifact_id=self.manifest['artifact_id'],source_result=str(self.result),quality_gate='SOURCE_REVIEW_REQUIRED',manual_corrections=self.manifest['stage']=='manual_review'),ensure_ascii=False,indent=2),encoding='utf-8')
            self.log.appendPlainText('내보내기: '+str(path))
        except (OSError,ValueError) as error:self.log.appendPlainText('내보내기 실패: '+str(error))

    def closeEvent(self,event):
        if self.process:
            self.stop();event.ignore();self.log.appendPlainText('분석을 중단하고 있습니다. 종료 후 창을 닫으세요.');return
        self.player.stop();event.accept()


def main():
    import argparse
    parser=argparse.ArgumentParser(description='CT Scene v2 review and full-source analysis')
    parser.add_argument('result',nargs='?');parser.add_argument('--front')
    args=parser.parse_args()
    app=QApplication(sys.argv[:1]);window=ReviewWindow()
    if args.result:window.load(args.result)
    if args.front:window.source.setText(args.front)
    window.show();return app.exec()


if __name__=='__main__':sys.exit(main())
