# GUNB — wzory formularzy

Oficjalne formularze GUNB / e-Budownictwo (proces budowlany).

## Oczekiwane pliki

- **`PB-5.pdf`** — oświadczenie o posiadanym prawie do dysponowania nieruchomością
  na cele budowlane (Dz.U. 2021 poz. 1170). Najlepiej **wypełnialny PDF (AcroForm)**.
  Używa go skill `wypelnianie-pb5`.

## Jak dodać PB-5

1. Pobierz wypełnialny wzór z e-Budownictwo / BIP urzędu i zapisz tutaj jako `PB-5.pdf`.
2. Sprawdź, czy ma pola do wypełnienia:
   ```bash
   python ".claude/skills/wypelnianie-pb5/scripts/inspect_fields.py" "Warsztat Architekta/Wzory/GUNB/PB-5.pdf"
   ```
3. Dalsze kroki (field_map, wypełnianie) — patrz `.claude/skills/wypelnianie-pb5/SKILL.md`.

Kolejne druki GUNB (PB-1, PB-2, PB-7…) dokładaj w tym folderze wg tej samej zasady.
