Running the CLE
===============

The CLE is not intended to be run in isolation and should be run through the NRP platform. For more information, please see the NRP Backend documentation.

However, you may want to run the unit tests and run the verification including PEP8 and pylint. To do so, the CLE root directory contains two scripts that you just need to invoke.

.. code-block:: bash

    ./run_tests.sh
    ./verify.sh

Note that verify does also invoke the tests.

You may also want to generate a fresh documentation. To do so, simply run

.. code-block:: bash

    make doc

after you have verified the code (Otherwise the generated documentation may be incomplete due to import errors). This documentation can be found in ``hbp_nrp_cle/doc/build/html``.
For example, this document will be generated to ``hbp_nrp_cle/doc/build/html/running.html``.