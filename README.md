## RANSAC Implementation in C++

This is a simple implementation of the RANSAC (Random Sampling Consensus) algorithm in C++. **RANSAC** is a robust method used for fitting models to data containing a significant number of outliers.

To test this algorithm, follow these steps:

1.  **Clone the Repository**

2.  **Make sure the follwing script is executable**
    ```bash
    run.sh
    ```

3.  **Use the script to run RANSAC Executables**

    * For **RANSAC Line** (fitting the equation of a line):
        ```bash
        ./run.sh RL
        ```

    * For **RANSAC Plane** (fitting the equation of a plane):
        ```bash
        ./run.sh RP
        ```

    * For **Both** :
        ```bash
        ./run.sh RL RP
        ```

Also check out my article where I explain the algorithm along with code bits: [Guide to Implementing RANSAC in C++](https://flashblog.hashnode.dev/guide-to-implementing-ransac-in-c-programming).


