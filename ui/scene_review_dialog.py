"""Inspect original pixels, references, detector outputs and reconstruction together."""
import json
from pathlib import Path
import cv2
import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage,QPixmap
from PyQt6.QtWidgets import (QDialog,QVBoxLayout,QHBoxLayout,QLabel,QComboBox,QCheckBox,
                            QTableWidget,QTableWidgetItem,QHeaderView,QSizePolicy)


def review_overlay(image,case,reference=True,detected=True,probe=False):
    canvas=image.copy()
    def box(bbox,color,text):
        x,y,w,h=map(round,bbox)
        cv2.rectangle(canvas,(x,y),(x+w,y+h),color,2)
        cv2.putText(canvas,text,(max(0,x),max(18,y-7)),0,.58,color,2,cv2.LINE_AA)
    if detected:
        for i,d in enumerate(case['probe' if probe else 'baseline']):
            score='REVIEW' if d.get('review_target') else f"{d['confidence']:.2f}"
            box(d['bbox'],(0,165,255),f"D{i} {d['kind']} {score}")
        for lane in case.get('observed_lanes',[]):
            cv2.polylines(canvas,[np.asarray(lane['points'],np.int32)],False,(70,220,70),3)
        for signal in case.get('observed_signals',[]):
            box(signal['bbox'],(80,255,255),signal.get('state','unknown').upper())
    if reference:
        matches={r['target_id']:r for r in case['probe_review' if probe else 'baseline_review']['results']}
        for target in case['targets']:
            missing=matches.get(target['id'],{}).get('status')=='missed'
            box(target['bbox'],(70,70,240) if missing else (240,225,0),target['id']+(' MISS' if missing else ' REF'))
        for lane in case['lanes']:
            cv2.polylines(canvas,[np.asarray(lane['points'],np.int32)],False,(255,80,255),3)
            cv2.putText(canvas,lane['id']+' MANUAL',tuple(lane['points'][0]),0,.5,(255,80,255),2)
        for signal in case['signals']: box(signal['bbox'],(80,255,255),signal['id']+' UNKNOWN')
    return canvas


class SceneReviewDialog(QDialog):
    def __init__(self,path,parent=None):
        super().__init__(parent)
        self.root=Path(path).resolve().parent
        self.report=json.loads(Path(path).read_text(encoding='utf-8'))
        if self.report.get('schema_version')!=1 or not self.report.get('cases'): raise ValueError('Invalid review package')
        self.setWindowTitle('검출물 원본 대조 / Detection Review'); self.resize(1320,900)
        layout=QVBoxLayout(self)
        self.summary=QLabel('선택 참조의 일대일 대응 검사입니다. 전체 정확도나 실측 3D 검증이 아닙니다. 청록=수동 참조 · 빨강=누락 · 주황=검출 · 보라=수동 차선 · 노랑=신호 판독 보류')
        self.summary.setWordWrap(True); layout.addWidget(self.summary)
        controls=QHBoxLayout()
        self.case=QComboBox()
        for c in self.report['cases']: self.case.addItem(f"{c['time_s']:.1f}초 / {c['camera']}")
        self.references=QCheckBox('수동 참조 표시'); self.references.setChecked(True)
        self.detections=QCheckBox('검출 표시'); self.detections.setChecked(True)
        self.probe=QCheckBox('신호등 클래스 포함 재검출 비교')
        for w in (self.case,self.references,self.detections,self.probe): controls.addWidget(w)
        layout.addLayout(controls)
        self.focus=QComboBox(); self.focus_key=None
        self.focus.currentIndexChanged.connect(self.refresh)
        zoom=QHBoxLayout(); zoom.addWidget(QLabel('원본 확대 영역')); zoom.addWidget(self.focus); layout.addLayout(zoom)
        images=QHBoxLayout(); self.original=QLabel(); self.scene=QLabel()
        for label in (self.original,self.scene):
            label.setMinimumSize(400,300); label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setSizePolicy(QSizePolicy.Policy.Ignored,QSizePolicy.Policy.Ignored); images.addWidget(label)
        layout.addLayout(images,1)
        self.note=QLabel(); self.note.setWordWrap(True); layout.addWidget(self.note)
        self.table=QTableWidget(); self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(['참조 ID','외형/설명','대응 결과','검출 종류','추적 ID','3D 배치'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setMaximumHeight(240); layout.addWidget(self.table)
        self.case.currentIndexChanged.connect(self.refresh)
        for control in (self.references,self.detections,self.probe): control.toggled.connect(self.refresh)
        self.refresh()

    def read_image(self,name):
        path=(self.root/name).resolve()
        if not path.is_relative_to(self.root): raise ValueError('Review image outside package')
        image=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_COLOR)
        if image is None: raise ValueError('Unreadable review image')
        return image

    @staticmethod
    def show_image(label,image):
        rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        pixmap=QPixmap.fromImage(QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],QImage.Format.Format_RGB888).copy())
        label.setPixmap(pixmap.scaled(label.size(),Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation))

    def refresh(self):
        case=self.report['cases'][self.case.currentIndex()]
        probe=self.probe.isChecked(); checked=case['probe_review' if probe else 'baseline_review']
        key=(self.case.currentIndex(),probe)
        if key!=self.focus_key:
            self.focus.blockSignals(True); self.focus.clear(); self.focus.addItem('전체 원본',None)
            for target in case['targets']+case['signals']: self.focus.addItem(target['id'],target['bbox'])
            for i,d in enumerate(case['probe' if probe else 'baseline']): self.focus.addItem(f"D{i} {d['kind']}",d['bbox'])
            self.focus.blockSignals(False); self.focus_key=key
        original=review_overlay(self.read_image(case['image']),case,self.references.isChecked(),self.detections.isChecked(),probe)
        bbox=self.focus.currentData()
        if bbox is not None:
            x,y,w,h=bbox; margin=max(w,h)*.25+15
            original=original[max(0,int(y-margin)):min(original.shape[0],int(y+h+margin)),
                              max(0,int(x-margin)):min(original.shape[1],int(x+w+margin))]
        self.show_image(self.original,original)
        self.show_image(self.scene,self.read_image(case['scene_image']))
        target_lookup={t['id']:t for t in case['targets']}
        self.table.setRowCount(len(checked['results']))
        for row,r in enumerate(checked['results']):
            target=target_lookup[r['target_id']]
            values=[r['target_id'],target.get('subtype',target.get('note','')), '대응' if r['status']=='matched' else '누락',
                    r.get('detected_kind') or '—',r.get('source_id') or '—',
                    '비교만 / 기존 재구성 유지' if probe else {'placed':'배치','not_placed':'보류'}.get(r.get('placement'),'없음')]
            for col,value in enumerate(values): self.table.setItem(row,col,QTableWidgetItem(value))
        judgements=case.get('judgements',[])
        text=f"선택 참조 {checked['targets']}개 중 {checked['matched']}개 대응, {checked['missed']}개 누락. 미대응 검출 {len(checked['unmatched_detections'])}개는 별도 검토 대상입니다."
        text+=f"\n자동 차선 구간 {len(case.get('observed_lanes',[]))}개(초록), 신호 후보 {len(case.get('observed_signals',[]))}개. 오른쪽은 표시 파이프라인의 3D 결과입니다."
        if case.get('review_assisted'):
            raw=case.get('raw_review',{})
            text+=f"\n검토 보정 사용: 자동 원시 검출 대응 {raw.get('matched',0)}/{raw.get('targets',0)}. REVIEW는 모델 확률이 아닙니다."
        relations=self.report.get('relations',{}).get(str(case['time_s']),[])
        if relations:
            counts={state:sum(r['outcome']==state for r in relations) for state in ('pass','fail','unassessable')}
            text+=f"\n좌우/앞뒤 순서: 대응 {counts['pass']}, 불일치 {counts['fail']}, 평가 불가 {counts['unassessable']} (실측 거리 검증 아님)."
        if judgements: text+='\n검토 메모: '+' / '.join(judgements)
        self.note.setText(text)

    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'table'): self.refresh()
