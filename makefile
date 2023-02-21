all:
	colcon build

	mkdir -p $(CURDIR)/install/uwb_sim/share/uwb_sim/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_sim/share/uwb_sim/config/anthor.xml

	mkdir -p $(CURDIR)/install/uwb_location/share/uwb_location/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_location/share/uwb_location/config/anthor.xml

sim:
	colcon build --packages-select uwb_interfaces uwb_sim

	mkdir -p $(CURDIR)/install/uwb_sim/share/uwb_sim/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_sim/share/uwb_sim/config/anthor.xml

interfaces:
	colcon build --packages-select uwb_interfaces

locate:
	colcon build --packages-select uwb_interfaces uwb_locate

	mkdir -p $(CURDIR)/install/uwb_locate/share/uwb_locate/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_locate/share/uwb_locate/config/anthor.xml

clean:
	rm -rf $(CURDIR)/build $(CURDIR)/install $(CURDIR)/log $(CURDIR)/launch/__pycache__