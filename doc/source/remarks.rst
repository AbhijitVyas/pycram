===============
General Remarks
===============

Jupyter Notebooks & PyCharm
===========================

If you have the situation that you activate your venv that has rospy installed and launched
PyCharm from the console you'll run into trouble using jupyter notebooks. PyCharm allows you
to correctly set up the jupyter server, however the notebook will always use your
``/usr/bin/python`` as interpreter and you can't select the correct one in the drop down menu.

To fix this issue one has to execute

.. code-block:: console

    python -m ipykernel install --user --name <kernel_name> --display-name "<Name_to_display>"

in your terminal. --name is the name of your virtual environment and --display-name is the name
that will display in the drop down menu of jupyter. After that, select the correct kernel and
everything should work now.

Adding Notebooks to the Documentation
=====================================

Adding notebooks to the documentation is done with the
`nbsphinx <https://docs.readthedocs.io/en/stable/guides/jupyter.html>`_ extension. If they are outside of the doc folder
please put a symbolic link in the doc/source/notebooks folder, such that no duplication is done. Sphinx will
automatically copy them during the build process. Use relative symbolic links since absolute paths won't work for
different machines.

Dirty Terminals
===============

If your terminal gets polluted by PyBullet complaining about incomplete URDF descriptions, you need to first fix your
URDF files by inserting the missing tags and second delete the `resources/cached` folder.