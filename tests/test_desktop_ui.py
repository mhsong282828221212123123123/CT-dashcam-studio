import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from datetime import datetime,timedelta
from PyQt6.QtCore import QSettings,Qt
from PyQt6.QtGui import QFontDatabase,QPalette
from PyQt6.QtWidgets import QApplication
from core import themes,i18n
from ui.main_window import CTDashcamStudio
from ui.dialogs import HotkeyGuideDialog
from ui.widgets import SingleCameraPopup

class DesktopUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QApplication.instance() or QApplication([])
        cls.app.setStyle('Fusion')
        if os.name=='nt':
            for font in ('malgun.ttf','segoeui.ttf','seguisym.ttf'):
                QFontDatabase.addApplicationFont('C:/Windows/Fonts/'+font)
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.settings=QSettings(str(Path(self.temp.name)/'settings.ini'),QSettings.Format.IniFormat)
        self.theme_patch=patch('core.themes._SETTINGS',lambda:self.settings);self.theme_patch.start()
        self.old_lang=i18n.get_current_language();self.old_settings=i18n._manager._settings
        i18n._manager._settings=self.settings;i18n.set_language('ko')
        themes.apply_theme('graphite',persist=False)
        with patch('ui.main_window.UpdateCheckWorker.start'):self.window=CTDashcamStudio()
        self.window.show();self.app.processEvents()
        root=Path(self.temp.name)/'TeslaCam';base=datetime(2026,10,30,12)
        for folder,count in [('RecentClips',23),('SavedClips/shared',3),('SavedClips/another/shared',2),('SentryClips/event',2)]:
            directory=root/folder;directory.mkdir(parents=True)
            for i in range(count):
                prefix=(base+timedelta(minutes=i)).strftime('%Y-%m-%d_%H-%M-%S')
                for cam in ('front','back'):(directory/(prefix+'-'+cam+'.mp4')).touch()
        self.window.scan_and_populate_tree(str(root));self.app.processEvents()
    def tearDown(self):
        self.window.close();self.app.processEvents()
        i18n._manager._current_lang=self.old_lang;i18n._manager._settings=self.old_settings
        self.theme_patch.stop();themes.apply_theme('graphite',persist=False);self.temp.cleanup()
    def recent(self):return self.window.tree_explorer.topLevelItem(0)
    def test_event_groups_and_duplicate_folder_names(self):
        tree=self.window.tree_explorer;self.assertEqual(tree.topLevelItemCount(),3)
        self.assertEqual([self.recent().child(i).childCount() for i in range(3)],[10,10,3])
        saved=tree.topLevelItem(1);self.assertEqual(saved.childCount(),2)
        self.assertEqual(sorted(len(saved.child(i).data(0,Qt.ItemDataRole.UserRole)['clip_list']) for i in range(2)),[2,3])
        self.assertTrue(saved.child(0).data(0,Qt.ItemDataRole.UserRole)['is_virtual_dir'])
        for index in range(saved.childCount()):
            folder=saved.child(index)
            self.assertEqual(folder.childCount(),len(folder.data(0,Qt.ItemDataRole.UserRole)['clip_list']))
            for child_index in range(folder.childCount()):
                data=folder.child(child_index).data(0,Qt.ItemDataRole.UserRole)
                self.assertFalse(data['is_group']);self.assertTrue(data['is_saved'])
                self.assertEqual(len(data['clip_list']),1)
        self.assertTrue(tree.topLevelItem(2).child(0).data(0,Qt.ItemDataRole.UserRole)['is_sentry'])
    def test_clip_type_colors_survive_theme_and_range_resets(self):
        tree=self.window.tree_explorer
        for theme in themes.PALETTES:
            self.window.combo_theme.setCurrentIndex(self.window.combo_theme.findData(theme))
            self.window._clear_tree_range_highlights()
            seen=[]
            for index,kind in enumerate(('recent','saved','sentry')):
                folder=tree.topLevelItem(index).child(0)
                self.assertEqual(folder.foreground(1).color().name(),themes.color('clip-'+kind))
                if kind!='sentry':
                    self.assertEqual(folder.child(0).foreground(1).color().name(),themes.color('clip-'+kind))
                seen.append(folder.foreground(1).color().name())
            self.assertEqual(len(set(seen)),3)

    def test_saved_folder_expands_without_loading_and_children_navigate(self):
        folder=self.window.tree_explorer.topLevelItem(1).child(0)
        with patch('ui.main_window.ClipLoaderWorker') as loader:
            self.window.on_tree_item_clicked(folder,0)
            loader.assert_not_called()
        self.assertTrue(folder.isExpanded())
        row=folder.child(0);self.window.tree_explorer.setCurrentItem(row)
        self.assertIs(self.window.get_sibling_clip_item(1),folder.child(1))

    def test_shared_typography_uses_bundled_font_and_default_spacing(self):
        from PyQt6.QtGui import QFontInfo
        from core.typography import SIZES, font_family
        self.assertEqual(font_family(),"Pretendard")
        self.assertEqual(self.app.font().family(),"Pretendard")
        for widget in (self.window.tree_explorer,self.window.btn_export,self.window.lbl_export_res,self.window.combo_lang):
            self.assertEqual(QFontInfo(widget.font()).family(),"Pretendard")
        self.assertEqual(self.app.font().pointSizeF(),SIZES["body"])
        self.assertEqual(self.app.font().letterSpacing(),100)
        self.assertEqual(self.app.font().wordSpacing(),0)
        self.assertTrue(self.app.font().kerning())
        for widget in self.app.allWidgets():
            template=widget.property('ctStyleTemplate') or ''
            self.assertNotIn('font-family:',template)
            self.assertNotIn('@font-',widget.styleSheet())

    def test_shortcut_descriptions_wrap_instead_of_clipping(self):
        from PyQt6.QtWidgets import QLabel
        dialog=HotkeyGuideDialog(self.window)
        dialog.show();self.app.processEvents()
        descriptions=dialog.findChildren(QLabel,'descLabel')
        self.assertEqual(len(descriptions),11)
        for label in descriptions:self.assertTrue(label.wordWrap())
        self.assertGreaterEqual(dialog.width(),640)
        dialog.close()

    def test_settings_and_palette(self):
        for name in themes.PALETTES:
            themes.apply_theme(name)
            self.assertEqual(self.settings.value('app_theme'),name)
            self.assertEqual(self.app.palette().color(QPalette.ColorRole.Window).name(),themes.PALETTES[name]['bg-app'])
        self.assertEqual(themes.resolve_theme('classic'),'midnight')
        for bad in (None,'unknown','',123):self.assertEqual(themes.resolve_theme(bad),'graphite')
    def test_geometry_and_playback_range_selection_preserved(self):
        w=self.window;group=self.recent().child(0);group.setExpanded(True);row=group.child(5)
        w.current_item=row;w.tree_explorer.setCurrentItem(row)
        w.active_clip_list=row.data(0,Qt.ItemDataRole.UserRole)['clip_list'];w.total_frames=720;w.fps=12
        w.slider.setRange(0,719);w.slider.blockSignals(True);w.slider.setValue(123);w.slider.blockSignals(False)
        stamp=datetime(2026,10,30,12,5)
        w.start_point={'dt':stamp,'frame':0,'item':row};w.end_point={'dt':stamp+timedelta(seconds=2),'frame':24,'item':row}
        w._refresh_playing_item();w.update_range_highlight();state=(w.start_point,w.end_point,w.active_clip_list)
        w.tree_explorer.verticalScrollBar().setValue(20);scroll=w.tree_explorer.verticalScrollBar().value();geometry=None
        for name in themes.PALETTES:
            w.combo_theme.setCurrentIndex(w.combo_theme.findData(name));self.app.processEvents()
            current=[v.geometry().getRect() for v in (w.tree_explorer,w.preview_lbl,w.btn_export,w.combo_theme,w.btn_play,w.slider)]
            geometry=geometry or current;self.assertEqual(current,geometry)
            self.assertIs(w.tree_explorer.currentItem(),row);self.assertTrue(group.isExpanded())
            self.assertEqual(w.tree_explorer.verticalScrollBar().value(),scroll)
            self.assertEqual(w.slider.value(),123);self.assertEqual((w.start_point,w.end_point,w.active_clip_list),state)
            self.assertTrue(row.text(0).startswith('▶ '));self.assertTrue(row.font(0).bold())
            self.assertEqual(row.background(0).color().name(),themes.color('warning-bg'))
    def test_navigation_and_range_cross_group_boundary(self):
        left=self.recent().child(0).child(9);right=self.recent().child(1).child(0)
        self.window.tree_explorer.setCurrentItem(left);self.assertIs(self.window.get_sibling_clip_item(1),right)
        self.window._apply_tree_range_highlights(datetime(2026,10,30,12,9),datetime(2026,10,30,12,11))
        for row in (left,right):self.assertEqual(row.background(0).color().name(),themes.color('warning-bg'))
        self.assertEqual(self.recent().child(1).child(1).background(0).style(),Qt.BrushStyle.NoBrush)
    def test_language_switch_preserves_links(self):
        row=self.recent().child(0).child(0);before=row.data(0,Qt.ItemDataRole.UserRole)
        themes.apply_theme('forest',persist=False)
        for lang in i18n.LANGUAGES:
            self.window.combo_lang.setCurrentIndex(self.window.combo_lang.findData(lang));self.app.processEvents()
            self.assertEqual(row.data(0,Qt.ItemDataRole.UserRole),before);self.assertEqual(themes.current_theme(),'forest')
            self.assertNotEqual(self.window.lbl_theme.text(),'lbl_theme')
    def test_minimum_window_controls_fit(self):
        self.window.resize(1150,680);self.app.processEvents();self.assertLessEqual(self.window.minimumSizeHint().width(),1150)
        for v in (self.window.btn_play,self.window.btn_prev,self.window.btn_next,self.window.btn_in,self.window.btn_out,self.window.btn_hotkeys):
            self.assertGreaterEqual(v.width(),v.minimumSizeHint().width())
    def test_theme_change_does_not_change_video_pixels(self):
        import numpy as np
        pixels=np.full((180,320,3),(30,80,140),dtype=np.uint8)
        self.window.last_grid_image=pixels.copy()
        self.window._display_grid_image(pixels)
        before=self.window.preview_lbl.pixmap().toImage()
        for name in themes.PALETTES:
            self.window.combo_theme.setCurrentIndex(self.window.combo_theme.findData(name))
            self.app.processEvents()
            self.assertEqual(self.window.preview_lbl.pixmap().toImage(),before)
            self.assertTrue(np.array_equal(self.window.last_grid_image,pixels))

    def test_open_popups_follow_palette(self):
        dialog=HotkeyGuideDialog(self.window);popup=SingleCameraPopup(self.window)
        dialog.show();popup.show();self.app.processEvents();themes.apply_theme('paper',persist=False)
        self.assertIn(themes.color('bg-app'),dialog.styleSheet());self.assertIn(themes.color('control-bg'),popup.styleSheet())
        self.assertNotIn('@ct-',dialog.styleSheet());dialog.close();popup.close()

    def test_paper_names_survive_language_changes(self):
        w = self.window
        w.combo_theme.setCurrentIndex(w.combo_theme.findData('paper-blue'))
        for lang in i18n.LANGUAGES:
            with self.subTest(language=lang):
                w.combo_lang.setCurrentIndex(w.combo_lang.findData(lang))
                self.app.processEvents()
                names = themes.theme_names(lang)
                self.assertEqual(list(names), list(themes.PALETTES))
                self.assertEqual(w.combo_theme.count(), 10)
                self.assertEqual([w.combo_theme.itemText(i) for i in range(10)], list(names.values()))
                self.assertEqual(w.combo_theme.currentData(), 'paper-blue')
        self.assertEqual(themes.theme_names('ko')['paper'], '페이퍼 그린')
        self.assertEqual(themes.theme_names('en')['paper'], 'Paper Green')

    def test_paper_text_and_clip_colors_remain_readable(self):
        def luminance(value):
            q = themes.QColor(value)
            linear = [v / 12.92 if v <= .04045 else ((v + .055) / 1.055) ** 2.4
                      for v in (q.redF(), q.greenF(), q.blueF())]
            return .2126 * linear[0] + .7152 * linear[1] + .0722 * linear[2]
        for name, palette in themes.PALETTES.items():
            if not name.startswith('paper'):
                continue
            for text, background in [('text-main', 'bg-app'), ('text-muted', 'bg-sidebar'),
                                     ('primary-text', 'primary-bg'), ('selected-text', 'selected-bg'),
                                     ('clip-recent', 'header-bg'), ('clip-saved', 'header-bg'), ('clip-sentry', 'header-bg')]:
                with self.subTest(theme=name, text=text):
                    a, b = luminance(palette[text]), luminance(palette[background])
                    self.assertGreaterEqual((max(a, b) + .05) / (min(a, b) + .05), 4.5)

if __name__=='__main__':unittest.main()
