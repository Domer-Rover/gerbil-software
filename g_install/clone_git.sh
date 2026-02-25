#!/bin/sh

alr_installed=""

if [ -n "$(find . -maxdepth 1 -type d -name "gerbil-software" -print -quit 2>/dev/null)" ]; then
    alr_installed="1"
    echo "You are all set with git cloning!"
    echo ""
    echo "Now you will need to install docker for your OS. Go to https://www.docker.com/get-started/ and get Docker. Once you finish setting it up, press 1 here in the terminal. "
    read reader_input
    clean_input=$(echo "$reader_input" | tr -cd '0-9')

    if [ "$reader_input" = "1" ]; then	    
	echo "Connecting to Docker (this might take a while for the first time)"
	echo ""
        docker compose up
	exit 0
    else
        echo "Input different than 1. Exiting."
        exit 1
    fi

    exit 0
else
    alr_installed="0"
    echo "Repo not cloned in you machine. Cloning repo..."
    echo ""
    git clone https://github.com/Domer-Rover/gerbil-software
    exit 1
fi
