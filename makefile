all:
	colcon build

	mkdir -p $(CURDIR)/install/uwb_simulation/share/uwb_simulation/config/
	cp $(CURDIR)/config/anchor.xml $(CURDIR)/install/uwb_simulation/share/uwb_simulation/config/anchor.xml

	mkdir -p $(CURDIR)/install/uwb_location/share/uwb_location/config/
	cp $(CURDIR)/config/anchor.xml $(CURDIR)/install/uwb_location/share/uwb_location/config/anchor.xml

simulation:
	colcon build --packages-select uwb_interfaces uwb_simulation

	mkdir -p $(CURDIR)/install/uwb_simulation/share/uwb_simulation/config/
	cp $(CURDIR)/config/anchor.xml $(CURDIR)/install/uwb_simulation/share/uwb_simulation/config/anchor.xml

interfaces:
	colcon build --packages-select uwb_interfaces

locate:
	colcon build --packages-select uwb_interfaces uwb_locate

	mkdir -p $(CURDIR)/install/uwb_locate/share/uwb_locate/config/
	cp $(CURDIR)/config/anchor.xml $(CURDIR)/install/uwb_locate/share/uwb_locate/config/anchor.xml

clean:
	rm -rf $(CURDIR)/build $(CURDIR)/install $(CURDIR)/log $(CURDIR)/launch/__pycache__