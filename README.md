# 3245_Limelight

Team 3245's experiment in computer vision with Limelight.

## Robot specifications

This program runs a KitBot with Talon SRX motor controllers connected with a CAN network. Please check the Drivetrain
subsystem for CAN IDs.

A simple wood block (or whatever) attached to a CIM motor is used to indicate when the Limelight has spotted a target.

## Limelight API

We've written a wrapper over the NetworkTables and JSON API. The advantage of using JSON dump output is that all
data is guaranteed to be collected in the same instant. The results from a single snapshot are contained in
a `Limelight.Result` object, which has plenty of accessors for getting data.

The wrapper currently only exposes data necessary for AprilTag tracking. We are considering adding support for custom
image recognition pipelines via classifiers and detectors in the future.
