#!/usr/bin/env python3
"""Wypisuje pola formularza (AcroForm) w pliku PDF.

Użycie:
    python scripts/inspect_fields.py templates/PB-5.pdf

Dla każdego pola podaje: nazwę, typ (/Tx, /Btn, ...) oraz — dla checkboxów —
dostępne stany „on" (potrzebne do field_map.json -> checkbox.on).
Jeśli plik NIE ma żadnych pól, to znaczy, że szablon jest „płaski" i wymaga
dodania warstwy pól (patrz SKILL.md → Przygotowanie szablonu).
"""
from __future__ import annotations

import sys
from pathlib import Path

from pypdf import PdfReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pdf_utils import checkbox_on_states  # noqa: E402


def main(path: str) -> int:
    reader = PdfReader(path)
    fields = reader.get_fields()
    if not fields:
        print(f"BRAK pól AcroForm w {path}.")
        print("=> Szablon jest 'płaski'. Dodaj warstwę pól zanim użyjesz fill_pb5.py.")
        return 2
    on_states = checkbox_on_states(reader)
    print(f"Znaleziono {len(fields)} pól w {path}:\n")
    for name, f in fields.items():
        ftype = f.get("/FT", "?")
        line = f"  {name!r:42} typ={ftype}"
        if str(ftype) == "/Btn" and name in on_states:
            line += f"  stany_on={on_states[name]}"
        print(line)
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("użycie: inspect_fields.py <plik.pdf>")
    raise SystemExit(main(sys.argv[1]))
