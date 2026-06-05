#!/usr/bin/env python3
"""Parser danych wejściowych Danet.txt dla oświadczenia PB-5.

Format tolerancyjny:
  - `klucz = wartość` lub `klucz: wartość`
  - sekcje: `[nazwa_sekcji]`
  - komentarze: linie zaczynające się od `#`
  - klucze powtarzalne (np. `dzialka`, `osoba`) zbierane są jako lista

Zwraca znormalizowany słownik (patrz funkcja `normalize`).
Uruchomiony bezpośrednio wypisuje sparsowaną strukturę jako JSON (do podglądu).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

# Dozwolone tytuły prawne (klucz -> indeks pola 1..6 wg PB-5)
TYTULY = {
    "wlasnosc": 1,
    "wspolwlasnosc": 2,
    "uzytkowanie_wieczyste": 3,
    "trwaly_zarzad": 4,
    "ograniczone_prawo_rzeczowe": 5,
    "stosunek_zobowiazaniowy": 6,
}

# Klucze, które mogą wystąpić wielokrotnie -> trafiają do list
LIST_KEYS = {"dzialka", "osoba"}


def _split_kv(line: str):
    """Rozbija linię na (klucz, wartość). Obsługuje '=' oraz ':'."""
    for sep in ("=", ":"):
        if sep in line:
            k, v = line.split(sep, 1)
            return k.strip().lower().replace(" ", "_"), v.strip()
    return None, None


def parse_raw(path: str | Path) -> dict:
    """Czyta plik do surowej struktury: {sekcja: {klucz: wartość | [wartości]}}."""
    text = Path(path).read_text(encoding="utf-8")
    data: dict[str, dict] = {"_root": {}}
    section = "_root"
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1].strip().lower()
            data.setdefault(section, {})
            continue
        key, val = _split_kv(line)
        if key is None:
            continue
        bucket = data.setdefault(section, {})
        if key in LIST_KEYS:
            bucket.setdefault(key, []).append(val)
        else:
            bucket[key] = val
    return data


def _parse_dzialka(s: str) -> dict:
    """'jednostka | obręb | nr' -> dict."""
    parts = [p.strip() for p in s.split("|")]
    parts += [""] * (3 - len(parts))
    return {"jednostka": parts[0], "obreb": parts[1], "nr_dzialki": parts[2]}


def _parse_osoba(s: str) -> dict:
    """'Imię Nazwisko | adres' -> dict (adres opcjonalny)."""
    parts = [p.strip() for p in s.split("|")]
    return {"nazwa": parts[0], "adres": parts[1] if len(parts) > 1 else ""}


def normalize(raw: dict) -> dict:
    """Sprowadza surową strukturę do modelu używanego przez fill_pb5."""
    root = raw.get("_root", {})
    inwestor = raw.get("inwestor", {})
    nieruch = raw.get("nieruchomosc", {})

    tytul = (nieruch.get("tytul_prawny") or root.get("tytul_prawny") or "").strip().lower()
    if tytul and tytul not in TYTULY:
        raise ValueError(
            f"Nieznany tytul_prawny={tytul!r}. Dozwolone: {', '.join(TYTULY)}"
        )

    dzialki = [_parse_dzialka(d) for d in nieruch.get("dzialka", [])]
    wspol = [_parse_osoba(o) for o in raw.get("wspolwlasciciele", {}).get("osoba", [])]
    sygn = [_parse_osoba(o)["nazwa"] for o in raw.get("sygnatariusze", {}).get("osoba", [])]

    model = {
        "organ": root.get("organ", ""),
        "inwestor": {
            "typ": inwestor.get("typ", "osoba fizyczna"),
            "imie_nazwisko": inwestor.get("imie_nazwisko") or inwestor.get("nazwa", ""),
            "nazwa": inwestor.get("nazwa", ""),
            "kraj": inwestor.get("kraj", "Polska"),
            "wojewodztwo": inwestor.get("wojewodztwo", ""),
            "powiat": inwestor.get("powiat", ""),
            "gmina": inwestor.get("gmina", ""),
            "miejscowosc": inwestor.get("miejscowosc", ""),
            "ulica": inwestor.get("ulica", ""),
            "nr_domu": inwestor.get("nr_domu", ""),
            "nr_lokalu": inwestor.get("nr_lokalu", ""),
            "kod_pocztowy": inwestor.get("kod_pocztowy", ""),
            "poczta": inwestor.get("poczta", ""),
            "reprezentant": inwestor.get("reprezentant", ""),
        },
        "dzialki": dzialki,
        "tytul_prawny": tytul,
        "tytul_prawny_indeks": TYTULY.get(tytul),
        "wspolwlasciciele": wspol,
        "sygnatariusze": sygn,
        "miejscowosc_podpisu": raw.get("sygnatariusze", {}).get("miejscowosc_podpisu", ""),
        "data_podpisu": raw.get("sygnatariusze", {}).get("data_podpisu", ""),
    }
    # Domyślny sygnatariusz = inwestor
    if not model["sygnatariusze"]:
        name = model["inwestor"]["imie_nazwisko"] or model["inwestor"]["nazwa"]
        if name:
            model["sygnatariusze"] = [name]
    return model


def validate(model: dict) -> list[str]:
    """Zwraca listę ostrzeżeń o brakujących/niespójnych danych (nie przerywa)."""
    warn = []
    if not model["organ"]:
        warn.append("Brak pola 'organ' (do kogo kierowane oświadczenie).")
    if not (model["inwestor"]["imie_nazwisko"] or model["inwestor"]["nazwa"]):
        warn.append("Brak nazwy/imienia i nazwiska inwestora.")
    if not model["dzialki"]:
        warn.append("Brak żadnej działki w sekcji [nieruchomosc].")
    if not model["tytul_prawny"]:
        warn.append("Brak pola 'tytul_prawny'.")
    if model["tytul_prawny"] == "wspolwlasnosc" and not model["wspolwlasciciele"]:
        warn.append("tytul_prawny=wspolwlasnosc, ale brak [wspolwlasciciele].")
    if not model["sygnatariusze"]:
        warn.append("Brak sygnatariuszy i nie da się ich wywieść z danych inwestora.")
    return warn


def load(path: str | Path) -> dict:
    return normalize(parse_raw(path))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("użycie: parse_danet.py <Danet.txt>")
    m = load(sys.argv[1])
    for w in validate(m):
        print(f"OSTRZEŻENIE: {w}", file=sys.stderr)
    print(json.dumps(m, ensure_ascii=False, indent=2))
