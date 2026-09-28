"""AI Delivery Drone Path Planner - Streamlit shell.

Streamlit only hosts the page; the map, the 3D A* planner and the UI run in
the browser (web/*.js). app.py inlines the web/ files into one HTML document
and injects the Mapbox token + defaults.
"""
import json, os
from pathlib import Path

import streamlit as st
import streamlit.components.v1 as components

WEB = Path(__file__).parent / "web"

st.set_page_config(page_title="AI Drone Path Planner", page_icon="🚁", layout="wide")
st.markdown(
    "<style>.block-container{padding:0 !important;max-width:100% !important}"
    "header,footer{visibility:hidden}iframe{border:0}</style>",
    unsafe_allow_html=True,
)


def get_token() -> str:
    try:
        if "MAPBOX_TOKEN" in st.secrets:
            return st.secrets["MAPBOX_TOKEN"]
    except Exception:  # no secrets file locally
        pass
    return os.environ.get("MAPBOX_TOKEN", "")


token = get_token()
if not token:
    token = st.text_input("Mapbox public token (pk...)", type="password",
                          help="Or set MAPBOX_TOKEN in .streamlit/secrets.toml")
if not token:
    st.info("Add a free Mapbox public token to start (https://account.mapbox.com/).")
    st.stop()


@st.cache_data
def build_html(token: str) -> str:
    cfg = {
        "token": token,
        "center": [78.3800, 17.4435],
        "startAddress": "Cyber Towers, HITEC City, Hyderabad",
        "destAddress": "Inorbit Mall, Madhapur, Hyderabad",
        "speed": 12,  # m/s, real cruise speed used for ETA
    }
    html = (WEB / "index.html").read_text(encoding="utf-8")
    for tag, name in (("/*STYLE*/", "style.css"), ("/*PLANNER*/", "planner.js"), ("/*APP*/", "app.js")):
        html = html.replace(tag, (WEB / name).read_text(encoding="utf-8"))
    return html.replace("__CONFIG__", json.dumps(cfg))


components.html(build_html(token), height=860, scrolling=False)
