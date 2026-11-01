"""Offline legal information only; no media/network initialization.

SPDX-License-Identifier: GPL-3.0-only
"""
from pathlib import Path

from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtGui import QDesktopServices
from PyQt6.QtWidgets import (QComboBox, QDialog, QHBoxLayout, QLabel, QPushButton,
                            QTabWidget, QTextBrowser, QVBoxLayout, QWidget)

from core.constants import APP_LICENSE, APP_VERSION
from core.i18n import tr
from core.themes import set_theme_style
from core.utils import resource_path


def legal_root():
    """Nuitka/PyInstaller resources first; ordinary source tree as fallback."""
    bundled = Path(resource_path('legal'))
    return bundled if (bundled / 'LICENSE').is_file() else Path(resource_path(''))


def read_legal_document(relative):
    root = legal_root().resolve()
    file = (root / relative).resolve()
    if not file.is_relative_to(root):
        raise ValueError('Document is outside legal resources')
    return file.read_text(encoding='utf-8-sig')


class LegalTextBrowser(QTextBrowser):
    def loadResource(self, kind, url):
        # Documents are fully bundled. Do not resolve remote images/styles or
        # relative assets automatically just because a Markdown file mentions one.
        return None


class LegalInfoDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(tr('legal_title'))
        self.setMinimumSize(640, 460)
        self.resize(850, 620)
        layout = QVBoxLayout(self)
        heading = QLabel(f'CT Dashcam Studio  ·  v{APP_VERSION}  ·  {APP_LICENSE}')
        heading.setWordWrap(True)
        layout.addWidget(heading)
        self.scope_label = QLabel(tr('legal_scope'))
        self.scope_label.setWordWrap(True)
        layout.addWidget(self.scope_label)
        note = QLabel(tr('legal_english_note'))
        note.setWordWrap(True)
        layout.addWidget(note)
        self.tabs = QTabWidget()
        self.browsers = []
        self.load_errors = []
        layout.addWidget(self.tabs, 1)
        self.tabs.addTab(self._browser('PRIVACY.md'), tr('legal_privacy'))
        self.tabs.addTab(self._browser('LICENSE', plain=True), tr('legal_project'))
        third_party = QWidget()
        third_layout = QVBoxLayout(third_party)
        self.document_selector = QComboBox()
        self.document_selector.setAccessibleName(tr('legal_document'))
        documents = [('THIRD_PARTY_NOTICES.md', tr('legal_third_party')),
                     ('LICENSE_AUDIT.md', tr('legal_audit')),
                     ('REDISTRIBUTION_REPORT.md', tr('legal_redistribution')),
                     ('SECURITY_FINDINGS.md', tr('legal_security'))]
        notice_dir = legal_root() / 'licenses'
        if notice_dir.is_dir():
            documents += [(file.relative_to(legal_root()).as_posix(), file.relative_to(notice_dir).as_posix())
                          for file in sorted(notice_dir.rglob('*')) if file.is_file()]
        for relative, title in documents:
            self.document_selector.addItem(title, relative)
        third_layout.addWidget(self.document_selector)
        self.third_party_browser = self._browser('THIRD_PARTY_NOTICES.md')
        third_layout.addWidget(self.third_party_browser, 1)
        self.document_selector.currentIndexChanged.connect(self._select_document)
        self.tabs.addTab(third_party, tr('legal_third_party'))
        row = QHBoxLayout()
        row.addStretch()
        close = QPushButton(tr('legal_close'))
        close.clicked.connect(self.accept)
        row.addWidget(close)
        layout.addLayout(row)
        set_theme_style(self, '''
            QDialog, QWidget { background-color:@ct-bg-app; color:@ct-text-main; }
            QLabel { color:@ct-text-muted; }
            QTextBrowser { background-color:@ct-bg-sidebar; color:@ct-text-main;
                border:1px solid @ct-control-border; border-radius:5px; padding:10px; }
            QPushButton, QComboBox { background-color:@ct-control-bg; color:@ct-text-main;
                border:1px solid @ct-control-border; border-radius:5px; padding:6px 12px; }
            QTabBar::tab { padding:8px 12px; }
            QTabBar::tab:selected { color:@ct-text-main; background-color:@ct-selected-bg; }
        ''')

    def _browser(self, relative, plain=False):
        browser = LegalTextBrowser()
        browser.setOpenExternalLinks(False)
        browser.setOpenLinks(False)
        browser.anchorClicked.connect(self._open_link)
        self.browsers.append(browser)
        self._load(browser, relative, plain)
        return browser

    def _load(self, browser, relative, plain=False):
        try:
            text = read_legal_document(relative)
        except (OSError, UnicodeError, ValueError):
            self.load_errors.append(relative)
            browser.setPlainText(f"{tr('legal_missing')}\n{relative}")
            return
        if plain:
            browser.setPlainText(text)
        else:
            browser.setMarkdown(text)
        browser.moveCursor(browser.textCursor().MoveOperation.Start)

    def _select_document(self):
        relative = self.document_selector.currentData()
        self._load(self.third_party_browser, relative,
                   plain=not relative.lower().endswith(('.md', '.rst')))

    def _open_link(self, url):
        if url.scheme() == 'https':
            QDesktopServices.openUrl(url)  # Explicit user click only.
        elif not url.scheme() and url.path():
            self._load(self.third_party_browser, url.path(), plain=not url.path().endswith('.md'))
            self.tabs.setCurrentIndex(2)
