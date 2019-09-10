# AESES

AES hardware implementation for the Embedded System course (2017/2018) and (2018/2019).
Target board: ***Digilent Nexys4 DDR rev. C*** (part no. *XC7A100TCSG324C-1*)

The `aeses_core` supports all the possible key-lengths, *AES-128*, *AES-192* and *AES-256*. Moreover, it allows to perform both encryption/decryption in *ECB* mode. Located on branch `master`.
- max working frequency: *166.67MHz*
- encryption/decryption in all modes performed in *15 clock cycles*
- utilization:
    - **Slice LUT** (12%)
    - **F8 Muxes** (6%)
    - **F7 Muxes** (6%)
    - **Slice register** (3%)

The `aeses_lite` is a lightweight version of the full core just featuring ECB encryption using *AES-256*. It is instead optimized for performance and footprint. Located on branch `aeses_lite`.
- max working frequency: *227.27MHz*
- encryption performed in *15 clock cycles*
- utilization:
    - **Slice LUT** (5%)
    - **F8 Muxes** (1%)
    - **F7 Muxes** (2%)
    - **Slice register** (1%)

Both of the designs are connected to the external world via a *UART*.

For the report, please refer to the `doc` folder in branch `master`

## Authors

Gabriele Termignone and Davide Sampietro
