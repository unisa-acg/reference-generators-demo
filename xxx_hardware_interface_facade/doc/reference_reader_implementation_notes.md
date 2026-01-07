# Reference Reader Implementation Notes

## Overview

The `ReferenceReader` class is designed to handle the reading of reference interfaces for both joint space and task space references.

This document provides an explanation of the implementation details and the rationale behind certain design decisions, particularly focusing on the `assign_command_interfaces` method.

> **Note**: What we call "reference interfaces" and "command interfaces" in this package are both instances of `CommandInterface` objects, as ROS does not provide a separate object to describe reference interfaces, and the two objects are structurally similar.

## `assign_command_interfaces` method

### Purpose

The `assign_command_interfaces` method is responsible for assigning command interfaces to the `ReferenceReader`.
This method is called once during the controller activation step and creates a new vector of command interfaces owned by the class based on the provided command interfaces and the corresponding underlying memory locations.

### Possible workarounds and challenges

The ideal solution would involve having just the `assign_loaned_command_interfaces(loaned_command_interfaces)` method in the base class, which would take only a vector of `hardware_interface::LoanedCommandInterface` as input.
However, a controller does not have access to the vector of loaned command interfaces for the references, which prevents us from using the same approach that works for the `CommandWriter` class.

Creating the vector of loaned command interfaces from the command interfaces generated in the `on_export_reference_generator` method does not work because the command interfaces are constantly deleted and re-instantiated by the `on_export_reference_generator` method, which is repeatedly called by the controller manager.
Additionally, it is not possible to reuse the same vector of `CommandInterface` by saving it as an instance variable.
Creating a deep copy of the command interfaces is not possible because the developers deliberately removed the copy constructor.

Working directly with the vector of double reference interfaces, plus all the information contained in the `CommandInterfaces`, would require creating a custom class or struct that holds this information, along with a double pointer, essentially replicating the `CommandInterface` to bypass the removal of the copy constructor.
This approach would lead to significant code duplication, which is why it was discarded.

### Current Solution

The current solution, although not ideal, works by creating a new vector of `CommandInterface` objects.
These objects copy the metadata from the `external_command_interfaces` vector and bind them with the same memory location as the external command interfaces, which are passed to this method as the `external_command_interfaces_data` vector.
This allows a vector of `LoanedCommandInterface` objects to be created from the copied `CommandInterface` objects.

This workaround is not ideal because it violates the `ros2_control` philosophy of ensuring unique ownership of command interfaces.
However, it addresses certain limitations in the ROS2 framework and ensures that the command interfaces are correctly assigned and used within the `ReferenceReader`.

### Future Improvements

In ROS2 Jazzy Jalisco, the `ros2_control` framework will handle reference interfaces of chainable controllers differently, by using shared pointers to the command interfaces.
This will allow the `ReferenceReader` to work with shared pointers to the command interfaces, which will simplify the implementation and avoid the need to create a new vector of `CommandInterface` objects.
