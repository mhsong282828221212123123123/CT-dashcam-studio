"""Legal UI/resource checks, with no updater or external network calls."""
import hashlib
import json
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from PyQt6.QtCore import QSettings, QUrl
from PyQt6.QtWidgets import QApplication
from core import i18n, themes
from core.constants import APP_LICENSE
from ui.legal_info import LegalInfoDialog, read_legal_document
from tools.prepare_legal_bundle import prepare, DOCUMENTS

ROOT = Path(__file__).resolve().parents[1]


class LegalInfoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.old_lang = i18n._manager._current_lang
        self.old_settings = i18n._manager._settings
        i18n._manager._settings = QSettings(str(Path(self.temp.name)/'prefs.ini'), QSettings.Format.IniFormat)

    def tearDown(self):
        i18n._manager._current_lang = self.old_lang
        i18n._manager._settings = self.old_settings
        self.temp.cleanup()

    def test_official_gpl_and_preserved_mit_rights(self):
        self.assertEqual(APP_LICENSE, 'GPL-3.0-only')
        self.assertEqual(hashlib.sha256((ROOT/'LICENSE').read_bytes()).hexdigest(),
                         '3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986')
        self.assertIn('Copyright (c) 2026 Companion Turret', read_legal_document('licenses/project-before-gpl-MIT.txt'))

    def test_five_languages_load_all_documents_without_network(self):
        keys = ['legal_title','legal_privacy','legal_project','legal_third_party','legal_english_note','legal_scope']
        with patch('ui.legal_info.QDesktopServices.openUrl') as open_url:
            for lang in i18n.LANGUAGES:
                i18n.set_language(lang)
                for key in keys:
                    self.assertIn(key, i18n.TRANSLATIONS[lang])
                dialog = LegalInfoDialog()
                self.assertEqual(dialog.windowTitle(), i18n.tr('legal_title'))
                self.assertEqual(dialog.tabs.count(), 3)
                self.assertIn('GPL-3.0-only', dialog.scope_label.text())
                for index in range(dialog.document_selector.count()):
                    dialog.document_selector.setCurrentIndex(index)
                    self.app.processEvents()
                self.assertEqual(dialog.load_errors, [])
                dialog.close()
            open_url.assert_not_called()

    def test_resource_traversal_is_rejected(self):
        with self.assertRaises(ValueError):
            read_legal_document('../outside.txt')

    def test_external_links_require_click_and_https(self):
        dialog = LegalInfoDialog()
        with patch('ui.legal_info.QDesktopServices.openUrl') as open_url:
            dialog._open_link(QUrl('http://example.test'))
            open_url.assert_not_called()
            dialog._open_link(QUrl('https://github.com/mhsong282828221212123123123/CT-dashcam-studio'))
            open_url.assert_called_once()
        dialog.close()

    def test_portable_legal_stage_contains_all_notices_and_hashes(self):
        stage = Path(self.temp.name)/'legal'
        result = prepare(stage)
        self.assertEqual(result['originalCodeLicense'], 'GPL-3.0-only')
        self.assertFalse(result['releaseClearance'])
        for name, expected in result['documents'].items():
            self.assertEqual(hashlib.sha256((stage/name).read_bytes()).hexdigest(), expected)
        for name in DOCUMENTS:
            self.assertTrue((stage/name).is_file())
        # Exercise the same reader against extracted/standalone legal resources.
        with patch('ui.legal_info.resource_path', lambda relative: str(stage) if relative == 'legal' else self.temp.name):
            dialog = LegalInfoDialog()
            self.assertEqual(dialog.load_errors, [])
            self.assertIn('GNU GENERAL PUBLIC LICENSE', dialog.browsers[1].toPlainText())
            dialog.close()

    def test_missing_resource_is_readable_error_instead_of_crash(self):
        with patch('ui.legal_info.read_legal_document', side_effect=OSError('missing')):
            dialog = LegalInfoDialog()
            self.assertIn('LICENSE', dialog.load_errors)
            self.assertIn(i18n.tr('legal_missing'), dialog.browsers[1].toPlainText())
            dialog.close()


if __name__ == '__main__':
    unittest.main()
