#!/usr/bin/env bash
cd "$(dirname "$0")" || exit 1
python3 -m pip install --user -r requirements.txt || exit 1
exec python3 main.py
