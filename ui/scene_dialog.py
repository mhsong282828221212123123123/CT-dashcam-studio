"""Experimental local analysis and inspection, isolated from playback."""
import json
from pathlib import Path
from datetime import datetime
import uuid
import cv2
import numpy as np
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer, QElapsedTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (QDialog,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,QSizePolicy,
                            QLineEdit,QFileDialog,QComboBox,QDoubleSpinBox,QProgressBar,QSlider,QWidget,QMessageBox)
from core.scene_analysis import create_scene_detector as SceneDetector
from core.scene_preview import EgoTimeline
from core.scene_pipeline import analyze, AnalysisCancelled
from core.driving_scene import DrivingRenderer, demo_scene
from core.scene_fusion import FusedSceneTimeline


class SceneWorker(QThread):
    progress=pyqtSignal(int)
    result=pyqtSignal(str)
    failed=pyqtSignal(str)
    cancelled=pyqtSignal()

    def __init__(self,front,output,model,size,start,duration,parent=None,review_reference=None,backend='cpu',detail='standard',road_map=None,road_model=None):
        super().__init__(parent)
        self.front,self.output,self.model,self.size=front,output,model,size
        self.start_s,self.duration=start,duration
        self.review_reference=review_reference
        self.backend,self.detail,self.road_map=backend,detail,road_map
        self.road_model=road_model

    def run(self):
        try:
            detector=SceneDetector(self.model or None,threshold=.25,input_size=self.size,include_traffic_lights=True,backend=self.backend,detail=self.detail)
            road_detector=None
            if self.road_model:
                from core.scene_road_model import RoadEvidenceDetector
                road_detector=RoadEvidenceDetector(self.road_model,'cpu' if self.backend=='cpu' else 'directml')
            analyze(self.front,self.output,sample_hz=10.,start=self.start_s,duration=self.duration,detector=detector,
                    progress=self.progress.emit,cancelled=self.isInterruptionRequested,make_video=True,review_reference=self.review_reference,road_map=self.road_map,road_detector=road_detector)
            self.result.emit(str(self.output))
        except AnalysisCancelled: self.cancelled.emit()
        except Exception as exc: self.failed.emit(str(exc))


class SceneExportWorker(QThread):
    progress=pyqtSignal(int)
    result=pyqtSignal(str)
    failed=pyqtSignal(str)
    cancelled=pyqtSignal()

    def __init__(self,timeline,output,parent=None,manifest=None,source_directory=None):
        super().__init__(parent);self.timeline=timeline;self.output=output
        self.overwrite=Path(output).exists()
        self.manifest,self.source_directory=manifest,source_directory

    def run(self):
        try:
            from core.scene_export import export_scene_video
            self.result.emit(export_scene_video(self.timeline,self.output,fps=30,
                progress=self.progress.emit,cancelled=self.isInterruptionRequested,overwrite=self.overwrite,
                manifest=self.manifest,source_directory=self.source_directory))
        except AnalysisCancelled:self.cancelled.emit()
        except Exception as exc:self.failed.emit(str(exc))


class SceneAnalysisDialog(QDialog):
    def __init__(self,front_path='',parent=None):
        super().__init__(parent)
        self.setWindowTitle('주행 분석 실험실 / Scene Lab'); self.resize(1140,820)
        self.worker,self.rows,self.result_dir=None,[],None
        self._pending_close=False
        self.renderer=DrivingRenderer(960,640)
        self.scene_timeline=None
        self.sample_hz=5.
        self.clock=QElapsedTimer()
        self.play_origin=0.
        self.current_time=0.
        self.timer=QTimer(self); self.timer.timeout.connect(self.next_frame)
        layout=QVBoxLayout(self)
        note=QLabel('전방·측방으로 위치와 움직임을 재구성합니다. 후방은 확인용 보조이며 위치를 변경하지 않습니다. 관측과 추정을 구분합니다.')
        note.setWordWrap(True); layout.addWidget(note)
        header=QHBoxLayout()
        self.setup_toggle=QPushButton('분석 설정'); self.setup_toggle.setCheckable(True)
        header.addWidget(self.setup_toggle)
        layout.addLayout(header)
        self.setup_panel=QWidget(); setup_layout=QVBoxLayout(self.setup_panel)
        setup_layout.setContentsMargins(0,0,0,0)
        layout.addWidget(self.setup_panel)
        self.setup_toggle.toggled.connect(self.setup_panel.setVisible)
        self.setup_panel.setVisible(False)
        row_layout=setup_layout
        def row(*widgets):
            line=QHBoxLayout()
            for widget in widgets: line.addWidget(widget)
            row_layout.addLayout(line)
        self.front=QLineEdit(front_path); self.front.setPlaceholderText('원본 *-front.mp4')
        self.input_button=QPushButton('원본 선택'); self.input_button.clicked.connect(self.choose_front)
        row(self.front,self.input_button)
        self.model=QLineEdit(); self.model.setPlaceholderText('비어 있으면 기본 YOLOv8n 320 사용')
        self.model_button=QPushButton('ONNX 모델 선택'); self.model_button.clicked.connect(self.choose_model)
        self.size=QComboBox(); self.size.addItems(['320','640','960'])
        candidate=Path(__file__).resolve().parents[2]/'scene-models'/'yolov8s.onnx'
        if candidate.is_file(): self.model.setText(str(candidate)); self.size.setCurrentText('640')
        row(self.model,self.model_button,self.size)
        self.inference_preset=QComboBox();self.inference_preset.addItem('직접 설정',None)
        for title,filename,size in [('GPU · 큰 모델 640','yolo11x.onnx',640),('GPU · 큰 모델 960','yolo11x-960.onnx',960),('GPU · 차량 윤곽 YOLO26m (실험)','model-review/yolo26m-seg.onnx',640),('GPU · 차량 윤곽 YOLO26m 960 (실험)','model-review/yolo26m-seg-960.onnx',960)]:
            path=candidate.parent/filename
            if path.is_file():self.inference_preset.addItem(title,(str(path),size))
        self.inference_preset.currentIndexChanged.connect(self.apply_inference_preset)
        row(QLabel('분석 프리셋'),self.inference_preset)
        self.road_model=QComboBox();self.road_model.addItem('기존 방식',None)
        for title,filename in [('YOLOP · 도로/차선 (실험)','yolop/yolop-640-640.onnx'),('YOLOPv2 · 도로/차선 (실험)','model-review/yolopv2-roads.onnx')]:
            path=candidate.parent/filename
            if path.is_file():self.road_model.addItem(title,str(path))
        row(QLabel('도로 관측 모델'),self.road_model)
        self.backend=QComboBox();self.backend.addItem('CPU','cpu');self.backend.addItem('GPU · DirectML (실패 시 CPU)','directml')
        self.detail=QComboBox();self.detail.addItem('전체 화면 분석','standard');self.detail.addItem('실험 · 전체 + 겹치는 4분할','tiled')
        row(QLabel('추론 장치'),self.backend,self.detail)
        self.road_map=QLineEdit();self.road_map.setPlaceholderText('선택: OSM 도로 JSON 캐시 (위치 정밀도는 별도 검증)')
        self.road_map_button=QPushButton('도로 데이터 선택');self.road_map_button.clicked.connect(self.choose_road_map)
        row(self.road_map,self.road_map_button)
        self.output=QLineEdit(str(Path(__file__).resolve().parents[2]/'scene-results'/'interactive'))
        self.output_button=QPushButton('결과 위치'); self.output_button.clicked.connect(self.choose_output)
        row(self.output,self.output_button)
        self.review_reference=QLineEdit(); self.review_reference.setPlaceholderText('선택 사항: 시각 검토 보정 참조 JSON (자동 검출과 별도 기록)')
        self.review_reference_button=QPushButton('검토 보정 참조'); self.review_reference_button.clicked.connect(self.choose_review_reference)
        row(self.review_reference,self.review_reference_button)
        self.start_time=QDoubleSpinBox(); self.start_time.setRange(0,86400); self.start_time.setSuffix(' 초부터')
        self.duration=QDoubleSpinBox(); self.duration.setRange(1,60); self.duration.setValue(10); self.duration.setSuffix(' 초 분석')
        self.run_button=QPushButton('분석 시작'); self.run_button.clicked.connect(self.start_analysis)
        self.cancel_button=QPushButton('분석 취소'); self.cancel_button.setEnabled(False); self.cancel_button.clicked.connect(self.cancel_analysis)
        self.load_button=QPushButton('기존 결과 열기'); self.load_button.clicked.connect(self.choose_result)
        row(self.start_time,self.duration,self.run_button,self.cancel_button)
        header.addWidget(self.load_button)
        self.review_button=QPushButton('검출 검토 열기'); self.review_button.clicked.connect(self.open_review)
        header.addWidget(self.review_button)
        self.export_button=QPushButton('3D + 4채널 영상 저장');self.export_button.setEnabled(False)
        self.export_button.clicked.connect(self.export_video);header.addWidget(self.export_button)
        self.progress=QProgressBar(); setup_layout.addWidget(self.progress)
        row_layout=layout
        self.status=QLabel('원본을 선택하거나 기존 분석 결과를 여세요.'); self.status.setWordWrap(True); layout.addWidget(self.status)
        self.view_mode=QComboBox(); self.view_mode.addItems(['실영상 · 카메라 통합','디자인 데모 · 합성','관측 영상'])
        self.camera_view=QComboBox(); self.camera_view.addItems(['all','front','back','left_repeater','right_repeater','left_pillar','right_pillar'])
        self.view_mode.currentIndexChanged.connect(self.change_view)
        self.setup_toggle.setChecked(bool(front_path))
        self.camera_view.currentTextChanged.connect(self.change_camera)
        row(QLabel('주행 시각화'),self.view_mode,QLabel('합성 범위 (all = 전체)'),self.camera_view)
        self.preview=QLabel(); self.preview.setMinimumSize(500,280); self.preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview.setSizePolicy(QSizePolicy.Policy.Ignored,QSizePolicy.Policy.Ignored)
        layout.addWidget(self.preview,1)
        self.ego=QLabel(); self.ego.setFixedSize(240,135)
        self.details=QLabel(); self.details.setWordWrap(True); row(self.ego,self.details)
        self.play_button=QPushButton('재생'); self.play_button.setEnabled(False); self.play_button.clicked.connect(self.toggle_play)
        self.slider=QSlider(Qt.Orientation.Horizontal); self.slider.valueChanged.connect(self.seek_frame)
        row(self.play_button,self.slider)
        self.edit_controls=[self.front,self.input_button,self.model,self.model_button,self.size,self.output,self.backend,self.detail,self.road_map,self.road_map_button,self.inference_preset,
                            self.output_button,self.review_reference,self.review_reference_button,self.start_time,self.duration,self.run_button,self.load_button,self.export_button,self.road_model]

    def export_video(self):
        if not self.scene_timeline or not self.rows or self.view_mode.currentIndex()!=0:return
        if self.worker and self.worker.isRunning():return
        source=Path(self.scene_manifest.get('source_front_path') or self.front.text())
        expected=self.scene_manifest.get('cameras',{}).get('front',{}).get('file')
        if not source.is_file() or source.name!=expected:
            path,_=QFileDialog.getOpenFileName(self,'하단 4채널에 사용할 원본 전방 영상 선택',self.front.text(),'MP4 (*.mp4)')
            if not path:return
            source=Path(path)
            if source.name!=expected:
                QMessageBox.warning(self,'원본 불일치','현재 분석과 같은 원본 전방 영상을 선택하세요.');return
        self.scene_manifest['source_front_path']=str(source.resolve())
        default=str(self.result_dir/'driving-scene.mp4')
        path,_=QFileDialog.getSaveFileName(self,'3D 주행 영상 저장 · 30fps',default,'MP4 (*.mp4)')
        if not path:return
        if not path.lower().endswith('.mp4'):path+='.mp4'
        self.timer.stop();self.play_button.setText('재생')
        self.worker=SceneExportWorker(self.scene_timeline,path,self,manifest=self.scene_manifest,source_directory=str(source.parent))
        self.worker.progress.connect(self.progress.setValue)
        self.worker.result.connect(lambda p:self.status.setText('3D 영상 저장 완료: '+p))
        self.worker.failed.connect(lambda text:self.status.setText('영상 저장 실패: '+text))
        self.worker.cancelled.connect(lambda:self.status.setText('영상 저장을 취소했습니다.'))
        self.worker.finished.connect(self.worker_finished)
        for control in self.edit_controls+[self.play_button,self.slider,self.view_mode,self.camera_view,self.review_button,self.export_button]:control.setEnabled(False)
        self.cancel_button.setEnabled(True);self.cancel_button.setText('저장 취소')
        self.progress.setValue(0);self.status.setText('3D 영상을 30fps MP4로 저장 중…')
        self.worker.start()

    def open_review(self):
        path,_=QFileDialog.getOpenFileName(self,'검출 검토 패키지의 review.json 선택',str(Path(__file__).resolve().parents[2]/'scene-results'),'JSON (*.json)')
        if not path: return
        try:
            from ui.scene_review_dialog import SceneReviewDialog
            SceneReviewDialog(path,self).exec()
        except Exception as exc: QMessageBox.warning(self,'검토 열기 실패',str(exc))

    def choose_review_reference(self):
        path,_=QFileDialog.getOpenFileName(self,'검토 보정 참조 JSON',str(Path(__file__).resolve().parents[2]/'scene-results'),'JSON (*.json)')
        if path: self.review_reference.setText(path)

    def choose_front(self):
        path,_=QFileDialog.getOpenFileName(self,'원본 전방 영상',self.front.text(),'MP4 (*.mp4)')
        if path: self.front.setText(path)

    def choose_model(self):
        path,_=QFileDialog.getOpenFileName(self,'YOLOv8 COCO ONNX',self.model.text(),'ONNX (*.onnx)')
        if path: self.model.setText(path)

    def apply_inference_preset(self):
        preset=self.inference_preset.currentData()
        if preset:
            path,size=preset;self.model.setText(path);self.size.setCurrentText(str(size))
            self.backend.setCurrentIndex(self.backend.findData('directml'));self.detail.setCurrentIndex(0)

    def choose_road_map(self):
        path,_=QFileDialog.getOpenFileName(self,'OSM 도로 JSON',self.road_map.text(),'JSON (*.json)')
        if path:self.road_map.setText(path)

    def choose_output(self):
        path=QFileDialog.getExistingDirectory(self,'결과 저장 위치',self.output.text())
        if path: self.output.setText(path)

    def choose_result(self):
        path=QFileDialog.getExistingDirectory(self,'manifest.json이 있는 결과 폴더',self.output.text())
        if path: self.load_result(path)

    def start_analysis(self):
        if self.worker and self.worker.isRunning(): return
        if not self.front.text().strip() or not self.output.text().strip():
            self.status.setText('원본 영상과 결과 위치를 지정하세요.'); return
        self.timer.stop(); self.play_button.setText('재생'); self.progress.setValue(0)
        self.rows=[]; self.preview.clear(); self.ego.clear(); self.details.clear(); self.play_button.setEnabled(False)
        directory=Path(self.output.text())/(datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
        self.worker=SceneWorker(self.front.text(),directory,self.model.text(),int(self.size.currentText()),
                                self.start_time.value(),self.duration.value(),self,review_reference=self.review_reference.text().strip() or None,
                                backend=self.backend.currentData(),detail=self.detail.currentData(),road_map=self.road_map.text().strip() or None,road_model=self.road_model.currentData())
        self.worker.progress.connect(self.progress.setValue); self.worker.result.connect(self.load_result)
        self.worker.failed.connect(lambda message:self.status.setText('분석 실패: '+message))
        self.worker.cancelled.connect(lambda:self.status.setText('분석을 취소했습니다. 원본은 변경되지 않았습니다.'))
        self.worker.finished.connect(self.worker_finished)
        for control in self.edit_controls: control.setEnabled(False)
        self.cancel_button.setEnabled(True); self.status.setText('로컬에서 분석 중… 완료 후 결과를 재생할 수 있습니다.')
        self.worker.start()

    def cancel_analysis(self):
        if self.worker and self.worker.isRunning():
            self.worker.requestInterruption(); self.status.setText('현재 처리 종료 후 취소합니다…')

    def worker_finished(self):
        if self.worker:
            self.worker.deleteLater()
            self.worker=None
        for control in self.edit_controls: control.setEnabled(True)
        self.cancel_button.setEnabled(False)
        self.cancel_button.setText('분석 취소')
        for control in [self.slider,self.view_mode,self.review_button]:control.setEnabled(True)
        self.play_button.setEnabled(bool(self.rows) or self.view_mode.currentIndex()==1)
        self.camera_view.setEnabled(self.view_mode.currentIndex()!=1)
        self.export_button.setEnabled(bool(self.rows) and self.view_mode.currentIndex()==0)
        if self._pending_close: super().reject()

    def load_result(self,directory):
        self.timer.stop(); self.play_button.setText('재생')
        try:
            root=Path(directory); manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
            rows=[json.loads(line) for line in (root/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
            if not rows: raise ValueError('결과가 비어 있습니다.')
            self.result_dir,self.rows=root,rows
            self.ego_timeline=EgoTimeline(rows, max_gap=1.5/float(manifest['sample_hz']))
            self.sample_hz=float(manifest['sample_hz'])
            camera_meta=manifest.get('cameras',{}).get(self.camera_view.currentText(),{})
            self.image_size=(camera_meta.get('width',1280),camera_meta.get('height',960))
            self.scene_manifest=manifest
            self.rebuild_scene()
            self.timer.setInterval(50)
            self.view_mode.setCurrentIndex(0)
            self.slider.setRange(0,len(rows)-1); self.slider.setValue(0); self.show_frame(0); self.play_button.setEnabled(True)
            inference=manifest.get('inference',{})
            reason=inference.get('fallback_reason')
            self.status.setText(f"{len(rows)}개 시점 · 입력 {manifest.get('model_input_size','?')} · {inference.get('effective','기존 CPU')} · {root}"+(' · CPU 전환: '+reason if reason else ''))
            self.export_button.setEnabled(True)
        except Exception as exc:
            self.rows=[]; self.play_button.setEnabled(False); self.status.setText('결과 열기 실패: '+str(exc))
            self.export_button.setEnabled(False)

    @staticmethod
    def pixmap(frame):
        rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        return QPixmap.fromImage(QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],QImage.Format.Format_RGB888).copy())

    def seek_frame(self,index):
        if self.timer.isActive():
            self.play_origin=index/20 if self.view_mode.currentIndex()==1 else self.rows[index]['time_s']
            self.clock.restart()
        self.show_frame(index)

    def rebuild_scene(self):
        camera=self.camera_view.currentText()
        rows=self.rows if camera=='all' else [dict(row,cameras={camera:row.get('cameras',{}).get(camera,{'available':False,'objects':[]})}) for row in self.rows]
        self.scene_timeline=FusedSceneTimeline(rows,self.scene_manifest)

    def change_camera(self):
        if self.rows: self.rebuild_scene()
        self.show_frame(self.slider.value())

    def change_view(self):
        self.timer.stop(); self.play_button.setText('재생')
        demo=self.view_mode.currentIndex()==1
        self.export_button.setEnabled(bool(self.rows) and self.view_mode.currentIndex()==0)
        self.camera_view.setEnabled(not demo)
        self.ego.setVisible(not demo)
        if demo: self.status.setText('합성 데모: 실제 영상과 무관한 동작으로 렌더링을 확인합니다.')
        elif self.result_dir: self.status.setText(str(self.result_dir))
        self.slider.blockSignals(True)
        self.slider.setRange(0,200 if demo else max(0,len(self.rows)-1)); self.slider.setValue(0)
        self.slider.blockSignals(False)
        self.play_button.setEnabled(demo or bool(self.rows))
        self.show_frame(0)

    def show_frame(self,index):
        if self.view_mode.currentIndex()==1:
            self.paint_scene(demo_scene(index/20))
            self.ego.clear()
            self.details.setText('합성 디자인 데모 — 실제 영상의 분석 결과가 아닙니다.\n추월 · 끼어들기 · 방향지시등 · 제동 · 곡선 도로를 확인하세요.')
            return
        if not self.rows or not 0 <= index < len(self.rows): return
        entry=self.rows[index]
        self.current_time=entry['time_s']
        path=self.result_dir/f'frame_{index:05d}.jpg'
        frame=cv2.imdecode(np.fromfile(str(path),dtype=np.uint8),cv2.IMREAD_COLOR) if path.is_file() else None
        scene=self.scene_timeline.at(index)
        scene_frame=self.renderer.render(scene)
        self.ego.setPixmap(self.pixmap(frame if frame is not None else scene_frame).scaled(self.ego.size(),Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation))
        image=self.pixmap(frame if self.view_mode.currentIndex()==2 and frame is not None else scene_frame)
        self.preview.setPixmap(image.scaled(self.preview.size(),Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation))
        descriptions=[f"시각 {entry['time_s']:.2f}초 · 배치 {len(scene.actors)}개 · 초기 투영 제외 {scene.rejected}개",
            f'전방·측방 중심 · 보조/중복 관측 {scene.fusion_count}개 결합 · 후방 위치 영향 없음',
            '가려진 접점·불명확한 배치는 제외 / 차선 폭과 차량 크기는 실측 아님']
        predictions=[a for a in scene.actors if a.tracking_state not in ('observed','reconstructed')]
        recovered=sum(a.tracking_state=='reconstructed' for a in scene.actors)
        descriptions.append(f'시간축 후보정 {recovered}개 · 재투영 검사 통과 / 추정 고스트로 표시')
        descriptions.append(f'일시 가림 예측 {len(predictions)}개 · 예측은 관측과 구분 / 불확실성·시간 초과 시 해제')
        descriptions.append(f"차선 표시 구간 {len(scene.lanes)}개 · 고스트=추정 차량 · 신호 후보 {len(getattr(scene,'signals',[]))}개 · 검토 보정 {'사용' if getattr(scene,'review_assisted',False) else '없음'}")
        road_status=getattr(scene,'road_status',{})
        if road_status.get('status')=='shape_prior':descriptions.append('OSM은 도로 형태 참고용 · 세부 차선은 카메라 관측 · 지도/영상 정렬은 미검증')
        indication=(getattr(scene,'road_context',None) or {}).get('junction_indication')
        if indication:
            descriptions.append('OSM 교차로: '+('영상·자차 움직임 근거로 확인' if indication['camera_confirmed'] else '지도 후보 · 영상 확인 대기')+' / GPS 근사 위치')
        elif road_status.get('status') in ('unmatched','unavailable'):
            descriptions.append('OSM 교차로: GPS/지도 매칭 불가')
        if scene.telemetry is None: descriptions.append('차량 SEI 없음 — 속도·등화·바퀴 움직임 미확정')
        for camera,data in entry['cameras'].items():
            count=sum(obj['track_status']=='confirmed' for obj in data['objects'])
            descriptions.append(f"{camera}: 반복 관측 {count}, 후보 {len(data['objects'])-count}" if data['available'] else f'{camera}: 읽을 수 없음')
        reasons={'grounded box fit unsupported':'잘림/차체 적합 불가','ground contact occluded':'접점 가림','ambiguous overlapping footprint':'겹침으로 위치 불확실'}
        rejected=self.scene_timeline.audit[index]['rejected']
        if rejected:
            descriptions.append('배치 보류: '+', '.join(item['id']+' ('+reasons.get(item['reason'],item['reason'])+')' for item in rejected))
        self.details.setText('\n'.join(descriptions))

    def paint_scene(self,scene):
        self.current_time=scene.time
        image=self.pixmap(self.renderer.render(scene))
        self.preview.setPixmap(image.scaled(self.preview.size(),Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation))

    def toggle_play(self):
        if self.timer.isActive(): self.timer.stop(); self.play_button.setText('재생')
        elif self.rows or self.view_mode.currentIndex()==1:
            if self.slider.value()==self.slider.maximum(): self.slider.setValue(0)
            self.play_origin=self.current_time
            self.clock.start(); self.timer.start(50); self.play_button.setText('일시정지')

    def next_frame(self):
        if self.slider.value()>=self.slider.maximum():
            self.timer.stop(); self.play_button.setText('재생'); return
        if not self.clock.isValid():
            self.slider.setValue(self.slider.value()+1); return
        timestamp=self.play_origin+self.clock.elapsed()/1000
        demo=self.view_mode.currentIndex()==1
        if demo:
            index=min(self.slider.maximum(),int(timestamp*20))
            scene=demo_scene(min(10.,timestamp))
        else:
            first=self.rows[0]['time_s']
            index=min(self.slider.maximum(),max(0,int((timestamp-first)*self.sample_hz)))
            scene=self.scene_timeline.sample(timestamp)
        previous_index=self.slider.value()
        self.slider.blockSignals(True); self.slider.setValue(index); self.slider.blockSignals(False)
        if not demo and index!=previous_index: self.show_frame(index)
        if demo or self.view_mode.currentIndex()==0: self.paint_scene(scene)
        else: self.show_frame(index)

    def reject(self):
        self.timer.stop()
        if self.worker and self.worker.isRunning():
            self._pending_close=True; self.cancel_analysis(); return
        super().reject()

    def closeEvent(self,event):
        if self.worker and self.worker.isRunning():
            self._pending_close=True; self.cancel_analysis(); event.ignore()
        else: self.timer.stop(); super().closeEvent(event)

    def resizeEvent(self,event):
        super().resizeEvent(event)
        if isinstance(getattr(self,'worker',None),SceneExportWorker) and self.worker.isRunning():return
        if hasattr(self,'slider') and (self.rows or self.view_mode.currentIndex()==1): self.show_frame(self.slider.value())
