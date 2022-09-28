all:
	colcon build

	mkdir -p $(CURDIR)/install/uwb_sim/share/uwb_sim/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_sim/share/uwb_sim/config/anthor.xml

	mkdir -p $(CURDIR)/install/uwb_location/share/uwb_location/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_location/share/uwb_location/config/anthor.xml

sim:
	colcon build --packages-select uwb_sim

	mkdir -p $(CURDIR)/install/uwb_sim/share/uwb_sim/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_sim/share/uwb_sim/config/anthor.xml

interfaces:
	colcon build --packages-select uwb_interfaces

location:
	colcon build --packages-select uwb_location

	mkdir -p $(CURDIR)/install/uwb_location/share/uwb_location/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_location/share/uwb_location/config/anthor.xml

clean:
	rm -rf $(CURDIR)/build $(CURDIR)/install $(CURDIR)/log