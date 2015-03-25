iCub models-consistency
======================

Set of tests meant to ensure consistency in kinematics model used
in iCub.

ikin-idyn-consistency
---------------------
This test will check that the end effector transforms (for the two hands and the two feet)
provided by the iKin chains and the iDyn iCubWholeBody object are consistent. While iDyn
is being discontinued, this check is important because URDF models for some model of iCub
(for example iCub v1) are generated from iDyn models.
