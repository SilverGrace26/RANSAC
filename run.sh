#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' 

rm -rf build
mkdir build && cd build
cmake ..
make

echo -e "${YELLOW}===============================${NC}"

if [ "$#" -eq 0 ]; then
    echo "Error: No arguments provided."
    echo "Usage: $0 <RL|RP|RL RP>"
    exit 1 
fi

if [ "$1" == "RL" ] && [ "$2" == "RP" ]; then
    echo "Both arguments detected. Running Ransac Line first..."
    
    ./RL
    
    # Check the exit status of the previous command ($?)
    if [ $? -eq 0 ]; then
        echo "Ransac Line completed successfully. Running Ransac Plane next..."
        echo -e "${GREEN}===============================${NC}"

        ./RP
        if [ $? -ne 0 ]; then
            echo "Error: Ransac Plane executable failed to run."
            echo -e "${RED}===============================${NC}"
        fi

        echo "Ransac Plane completed successfully."
        echo -e "${GREEN}===============================${NC}"
    else
        echo "Error: Ransac Line executable failed to run. Not proceeding with Ransac Plane."
        echo -e "${RED}===============================${NC}"
    fi
    
    exit 0 
fi

if [ "$#" -eq 1 ]; then
    if [ "$1" == "RL" ]; then
        echo "Running Ransac Line..."
        ./RL

        echo "Ransac Line completed successfully." 
        echo -e "${GREEN}===============================${NC}"

        if [ $? -ne 0 ]; then
            echo "Error: Ransac Line executable failed to run."
            echo -e "${RED}===============================${NC}"
        fi
        exit 0
    elif [ "$1" == "RP" ]; then
        echo "Running Ransac Plane..."
        ./RP

        echo "Ransac Plane completed successfully." 
        echo -e "${GREEN}===============================${NC}"

        if [ $? -ne 0 ]; then
            echo "Error: Ransac Plane executable failed to run."
            echo -e "${RED}===============================${NC}"
        fi
        exit 0
    fi
fi

echo -e "${RED}===============================${NC}"
echo -e "${RED}Error: Invalid arguments provided.${NC}" 
echo "Usage: $0 <RL|RP|RL RP>"
exit 1 
