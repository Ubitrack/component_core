utcomponents
============
This is the utcomponents Ubitrack submodule.

Description
----------
The utcomponents contains contains basic dataflow modules. These cover most common tracking and registration cases.

Usage
-----
In order to use it, you have to clone the buildenvironment, change to the ubitrack directory and add the utcomponents by executing:

    git submodule add https://github.com/Ubitrack/utcomponents.git modules/utcomponents


Dependencies
----------
In addition, this module has to following submodule dependencies which have to be added for successful building:

<table>
  <tr>
    <th>Component</th><th>Dependency</th>
  </tr>
  <tr>
    <td>all</td><td>utDataflow</td>
  </tr>
</table>
