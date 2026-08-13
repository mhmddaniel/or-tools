FROM python:3.12-slim

COPY . /srv/flask_app
WORKDIR /srv/flask_app

RUN apt-get update && apt-get clean

RUN pip install --no-cache-dir -r requirements.txt

EXPOSE 8000

CMD ["gunicorn", "--bind", "0.0.0.0:8000", "wsgi:app"]
