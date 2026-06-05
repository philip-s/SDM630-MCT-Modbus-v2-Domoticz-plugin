#!/usr/bin/env python3
"""Pomocnicze funkcje PDF/AcroForm wspólne dla inspect_fields i fill_pb5."""
from __future__ import annotations

from pypdf import PdfReader
from pypdf.generic import DictionaryObject


def checkbox_on_states(reader: PdfReader) -> dict[str, list[str]]:
    """Zwraca {nazwa_pola: [stany 'on']} dla checkboxów/przycisków.

    Stany odczytywane są z /AP/N widgetów (na poziomie adnotacji strony),
    bo na poziomie pola /AP często nie występuje. Pomija stan '/Off'.
    """
    states: dict[str, set] = {}
    for page in reader.pages:
        for annot in page.get("/Annots", []) or []:
            obj = annot.get_object()
            name = obj.get("/T")
            # widget może dziedziczyć nazwę z rodzica
            parent = obj.get("/Parent")
            if name is None and parent is not None:
                name = parent.get_object().get("/T")
            if name is None:
                continue
            ap = obj.get("/AP")
            if not isinstance(ap, DictionaryObject):
                continue
            n = ap.get("/N")
            if not isinstance(n, DictionaryObject):
                continue
            on = {str(k) for k in n.keys() if str(k) != "/Off"}
            if on:
                states.setdefault(str(name), set()).update(on)
    return {k: sorted(v) for k, v in states.items()}
