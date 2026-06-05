#!/usr/bin/env python3
"""Wypełnia oświadczenie PB-5 (wypełnialny PDF / AcroForm) danymi z Danet.txt.

Generuje JEDEN plik PDF na sygnatariusza (jedno oświadczenie = jeden podpis).
Podpisu NIE składa — to robi inwestor (odręcznie lub podpisem elektronicznym).

Użycie:
    python scripts/fill_pb5.py \
        --danet Danet.txt \
        --template templates/PB-5.pdf \
        --field-map templates/field_map.json \
        --out out/

Wymaga: pypdf. Pola z field_map, których nie ma w PDF, są pomijane (z ostrzeżeniem).
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

from pypdf import PdfReader, PdfWriter
from pypdf.generic import NameObject

sys.path.insert(0, str(Path(__file__).resolve().parent))
from parse_danet import load as load_danet, validate  # noqa: E402
from pdf_utils import checkbox_on_states  # noqa: E402


def slugify(s: str) -> str:
    s = s.strip().lower()
    s = (s.replace("ą", "a").replace("ć", "c").replace("ę", "e").replace("ł", "l")
           .replace("ń", "n").replace("ó", "o").replace("ś", "s").replace("ż", "z")
           .replace("ź", "z"))
    s = re.sub(r"[^a-z0-9]+", "_", s).strip("_")
    return s or "sygnatariusz"


def build_logical_values(model: dict, signer: str, warnings: list[str]) -> dict:
    """Buduje słownik {logiczna_nazwa: wartość} dla jednego sygnatariusza."""
    inv = model["inwestor"]
    typ = (inv.get("typ") or "").lower()

    vals: dict[str, str] = {
        "organ": model["organ"],
        "inwestor_nazwa": inv["imie_nazwisko"] or inv["nazwa"],
        "inwestor_kraj": inv["kraj"],
        "inwestor_wojewodztwo": inv["wojewodztwo"],
        "inwestor_powiat": inv["powiat"],
        "inwestor_gmina": inv["gmina"],
        "inwestor_miejscowosc": inv["miejscowosc"],
        "inwestor_ulica": inv["ulica"],
        "inwestor_nr_domu": inv["nr_domu"],
        "inwestor_nr_lokalu": inv["nr_lokalu"],
        "inwestor_kod_pocztowy": inv["kod_pocztowy"],
        "inwestor_poczta": inv["poczta"],
        "inwestor_reprezentant": inv["reprezentant"],
        "podpis_imie": signer,
    }

    # data + miejscowość podpisu
    md = " ".join(x for x in (model.get("miejscowosc_podpisu", ""),
                              model.get("data_podpisu", "")) if x).strip()
    vals["miejscowosc_data"] = md

    # współwłaściciele -> jeden tekst
    if model["wspolwlasciciele"]:
        vals["wspolwlasciciele"] = "; ".join(
            f"{o['nazwa']}" + (f", {o['adres']}" if o["adres"] else "")
            for o in model["wspolwlasciciele"]
        )

    # checkboxy: typ podmiotu
    if "fizyczn" in typ:
        vals["typ_osoba_fizyczna"] = True
    elif "prawn" in typ:
        vals["typ_osoba_prawna"] = True
    elif "jednost" in typ:
        vals["typ_jednostka"] = True

    # checkboxy: tytuł prawny (1..6)
    idx = model.get("tytul_prawny_indeks")
    tytul_key = {
        1: "tytul_1_wlasnosc", 2: "tytul_2_wspolwlasnosc",
        3: "tytul_3_uzytkowanie_wieczyste", 4: "tytul_4_trwaly_zarzad",
        5: "tytul_5_ograniczone_prawo_rzeczowe", 6: "tytul_6_stosunek_zobowiazaniowy",
    }.get(idx)
    if tytul_key:
        vals[tytul_key] = True

    return vals


def add_table(vals: dict, model: dict, max_rows: int, warnings: list[str]) -> None:
    dz = model["dzialki"]
    if len(dz) > max_rows:
        warnings.append(
            f"Działek: {len(dz)}, a szablon ma {max_rows} wierszy. "
            f"Nadmiarowe trzeba dołączyć jako osobny załącznik."
        )
    for i, d in enumerate(dz[:max_rows], start=1):
        vals[f"dzialka_{i}_jednostka"] = d["jednostka"]
        vals[f"dzialka_{i}_obreb"] = d["obreb"]
        vals[f"dzialka_{i}_nr"] = d["nr_dzialki"]


def _on_value(target: str, spec: dict, on_states: dict) -> NameObject:
    """Ustala stan 'on' checkboxa: priorytet ma realny stan z PDF, potem field_map.

    Wartość checkboxa MUSI być NameObject z ukośnikiem (np. /Yes), inaczej pypdf
    pozostawi pole odznaczone.
    """
    detected = on_states.get(target)
    if detected:
        name = detected[0]
    else:
        name = spec.get("on", "Yes")
    if not name.startswith("/"):
        name = "/" + name
    return NameObject(name)


def to_pdf_fields(logical: dict, fmap: dict, pdf_fields: set,
                  on_states: dict, warnings: list[str]) -> dict:
    """Tłumaczy {logiczna: wartość} na {nazwa_pola_pdf: wartość} wg field_map."""
    out: dict[str, object] = {}
    text_map = fmap.get("text", {})
    cb_map = fmap.get("checkbox", {})
    missing = set()

    for lname, value in logical.items():
        if value in (None, "", False):
            continue
        if lname in text_map:
            target = text_map[lname]
            if target in pdf_fields:
                out[target] = str(value)
            else:
                missing.add(target)
        elif lname in cb_map:
            spec = cb_map[lname]
            target = spec["field"]
            if target in pdf_fields:
                out[target] = _on_value(target, spec, on_states)
            else:
                missing.add(target)
    if missing:
        warnings.append(
            "Pola z field_map nieobecne w PDF (pominięto): " + ", ".join(sorted(missing))
        )
    return out


def fill_one(template: str, values: dict, out_path: Path) -> None:
    reader = PdfReader(template)
    writer = PdfWriter()
    writer.append(reader)
    for page in writer.pages:
        writer.update_page_form_field_values(page, values, auto_regenerate=False)
    # wymuś renderowanie wartości przez czytniki PDF
    try:
        writer.set_need_appearances_writer(True)
    except Exception:
        pass
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as fh:
        writer.write(fh)


def main() -> int:
    ap = argparse.ArgumentParser(description="Wypełnia PB-5 z Danet.txt")
    ap.add_argument("--danet", required=True)
    ap.add_argument("--template", required=True)
    ap.add_argument("--field-map", required=True)
    ap.add_argument("--out", default="out")
    args = ap.parse_args()

    model = load_danet(args.danet)
    warnings = list(validate(model))

    fmap = json.loads(Path(args.field_map).read_text(encoding="utf-8"))
    reader = PdfReader(args.template)
    pdf_fields = set((reader.get_fields() or {}).keys())
    if not pdf_fields:
        print("BŁĄD: szablon nie ma pól AcroForm (jest 'płaski'). "
              "Najpierw dodaj warstwę pól — patrz SKILL.md.", file=sys.stderr)
        return 2
    on_states = checkbox_on_states(reader)

    max_rows = fmap.get("table", {}).get("max_rows", 4)
    out_dir = Path(args.out)
    produced = []

    for signer in model["sygnatariusze"]:
        logical = build_logical_values(model, signer, warnings)
        add_table(logical, model, max_rows, warnings)
        values = to_pdf_fields(logical, fmap, pdf_fields, on_states, warnings)
        out_path = out_dir / f"PB-5_{slugify(signer)}.pdf"
        fill_one(args.template, values, out_path)
        produced.append(out_path)

    # de-duplikacja ostrzeżeń, zachowując kolejność
    seen = set()
    for w in warnings:
        if w not in seen:
            print(f"OSTRZEŻENIE: {w}", file=sys.stderr)
            seen.add(w)

    for p in produced:
        print(f"zapisano: {p}")
    print(f"\nGotowe: {len(produced)} plik(ów). Dokumenty wymagają podpisu inwestora.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
