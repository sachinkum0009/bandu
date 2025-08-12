# Installation

The installation steps for the Bandu application are straightforward. Follow these steps to set up your environment and run the application.

> Note: Follow commands carefully

## Prerequisites

```bash
# create a virtual env
python -m venv venv
source venv/bin/activate  # On Windows use `venv\Scripts\activate`

# install dependencies
pip install -r requirements.txt

# run the application
chainlit run app.py
```

It will start the localhost server at 8000 port. Then you can access the application at `http://localhost:8000` in your web browser.
