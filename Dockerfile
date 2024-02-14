FROM ubuntu:20.04
LABEL name = "Mads Rossen"
LABEL mail = "madsrossen@me.com"
LABEL version = "0.1"

# Set the locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8