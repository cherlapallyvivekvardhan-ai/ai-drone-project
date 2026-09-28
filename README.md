AI-Based Delivery Drone Path Planning System

A Streamlit-based AI drone path-planning application that provides:

Interactive 2D map selection

Start and destination point selection

3D drone flight visualization

AI-based path planning

Building and obstacle handling

Drone altitude and physics controls

Wind and battery-related simulation

Dynamic route rerouting

OpenStreetMap building data

Animated 3D flight viewer

Project Structure

ai-drone-project/
├── app.py
├── sim3d.py
├── viewer.py
├── requirements.txt
└── README.md

Requirements

The application uses these Python packages:

Streamlit

Folium

Streamlit-Folium

Pandas

The sim3d.py module uses Python standard-library modules and does not require additional third-party packages.

Run Locally

Open a terminal in the project folder and run:

pip install -r requirements.txt
streamlit run app.py

After starting, Streamlit will provide a local URL in the terminal, normally:

http://localhost:8501

Open that address in your browser.

Deploy on Streamlit Community Cloud

Upload app.py, sim3d.py, viewer.py, requirements.txt, and README.md to your GitHub repository.

Open Streamlit Community Cloud.

Create/select the app from your GitHub repository.

Set the main file to:

app.py

Deploy the application.

Streamlit Cloud will install the packages listed in requirements.txt before running the application.

Important

Keep requirements.txt in the same GitHub repository and at the project root, alongside app.py.

If you add another third-party Python package to the code later, add that package to requirements.txt before redeploying.
