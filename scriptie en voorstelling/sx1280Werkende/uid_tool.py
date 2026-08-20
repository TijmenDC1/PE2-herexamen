#!/usr/bin/env python3
"""
ELRS UID gereedschap
====================

Twee dingen:

1) binding phrase  ->  UID bytes
   ExpressLRS hasht je binding phrase met MD5 en neemt de eerste 6 bytes.
   Er doen twee varianten de ronde over WAT er precies gehasht wordt, dus
   dit script rekent ze allebei uit. Vergelijk met de UID die je al hebt om
   te zien welke variant jouw firmware gebruikt.

2) UID + OTA-versie  ->  CRC-seed en FHSS-seed
   Zodat je meteen ziet wat er in sx1280include.h hoort te staan, en of dat
   klopt met wat de ontvanger uit de lucht meet.

Gebruik:
    python uid_tool.py "mijn binding phrase"
    python uid_tool.py --uid 163,59,219,118,199,162
    python uid_tool.py --zoek 0x05          (welke UID4/versie past bij de meting)
"""

import hashlib
import sys


def uid_van_phrase(phrase):
    """Beide varianten die in omloop zijn."""
    met_define = hashlib.md5(
        ('-DMY_BINDING_PHRASE="' + phrase + '"').encode()
    ).digest()[:6]
    kaal = hashlib.md5(phrase.encode()).digest()[:6]
    return list(met_define), list(kaal)


def seeds(uid, ota_versie):
    """Zelfde formules als in sx1280include.h / ELRS OTA.cpp."""
    crc_init = ((uid[4] << 8) | uid[5]) ^ (ota_versie << 8)
    fhss_seed = ((uid[2] << 24) | (uid[3] << 16) |
                 (uid[4] << 8) | (uid[5] ^ ota_versie))
    return crc_init, fhss_seed


def toon(uid, label=""):
    print(f"\nUID {label}: {uid}")
    print(f"  hex: {' '.join(f'{b:02X}' for b in uid)}")
    print()
    print("  ver | CRC seed | bits[13:8] | FHSS seed  | opmerking")
    print("  ----+----------+------------+------------+-----------------------")
    for v in (3, 4):
        crc_init, fhss = seeds(uid, v)
        hoog = (crc_init >> 8) & 0x3F
        opm = "ELRS 3.x" if v == 3 else "ELRS 4.x"
        print(f"   {v}  |  0x{crc_init:04X}  |    0x{hoog:02X}     "
              f"| 0x{fhss:08X} | {opm}")
    print()
    print("  Let op: bits 14 en 15 van de CRC-seed doen niet mee in de CRC14,")
    print("  dus 'bits[13:8]' is wat je ontvanger uit de lucht kan meten.")


def zoek(gemeten_hoog):
    """Welke combinaties van UID4 en OTA-versie geven deze gemeten waarde?"""
    print(f"\nGemeten bits[13:8] van de seed = 0x{gemeten_hoog:02X}")
    print("Dit zijn alle UID[4]-waarden die daarbij passen:\n")
    print("  OTA-versie | mogelijke UID[4]")
    print("  -----------+--------------------------")
    for v in (3, 4):
        laag6 = gemeten_hoog ^ (v & 0x3F)
        kandidaten = [laag6 + 64 * k for k in range(4)]
        print(f"      {v}      | {kandidaten}")
    print()
    print("  UID[5] volgt hier NIET uit: die zit in de lage byte van de seed,")
    print("  en die wordt bij RC-pakketten met de nonce geXORd. Alleen een")
    print("  echt SYNC-pakket geeft UID[5] rechtstreeks.")


def main():
    args = sys.argv[1:]

    if not args:
        print(__doc__)
        print("\n--- huidige instelling in sx1280include.h ---")
        toon([163, 59, 219, 118, 199, 162], "(nu in de code)")
        zoek(0x05)
        return

    if args[0] == "--uid":
        uid = [int(x) for x in args[1].replace(" ", "").split(",")]
        if len(uid) != 6:
            sys.exit("UID moet 6 getallen zijn")
        toon(uid)
        return

    if args[0] == "--zoek":
        zoek(int(args[1], 0))
        return

    phrase = " ".join(args)
    met_define, kaal = uid_van_phrase(phrase)
    print(f'\nBinding phrase: "{phrase}"')
    print("\nVariant A - md5 van '-DMY_BINDING_PHRASE=\"<phrase>\"'")
    toon(met_define, "(variant A)")
    print("\nVariant B - md5 van de kale phrase")
    toon(kaal, "(variant B)")
    print("\nWelke van de twee overeenkomt met de UID die je al kent, dat is")
    print("de variant die jouw firmware gebruikt. Kloppen ze geen van beide,")
    print("gebruik dan de officiele generator:")
    print("  https://www.expresslrs.org/hardware/spi-receivers/#uid-byte-generator")


if __name__ == "__main__":
    main()
