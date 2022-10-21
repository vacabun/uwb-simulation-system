all:
	colcon build

	mkdir -p $(CURDIR)/install/uwb_sim/share/uwb_sim/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_sim/share/uwb_sim/config/anthor.xml

	mkdir -p $(CURDIR)/install/uwb_location/share/uwb_location/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_location/share/uwb_location/config/anthor.xml

	mkdir -p $(CURDIR)/install/uwb_sim_3d/share/uwb_sim_3d/config/
	cp $(CURDIR)/config/anthor_3d.xml $(CURDIR)/install/uwb_sim_3d/share/uwb_sim_3d/config/anthor.xml
	
	mkdir -p $(CURDIR)/install/uwb_location_3d/share/uwb_location_3d/config/
	cp $(CURDIR)/config/anthor_3d.xml $(CURDIR)/install/uwb_location_3d/share/uwb_location_3d/config/anthor.xml

sim:
	colcon build --packages-select uwb_sim

	mkdir -p $(CURDIR)/install/uwb_sim/share/uwb_sim/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_sim/share/uwb_sim/config/anthor.xml

sim_3d:
	colcon build --packages-select uwb_sim_3d

	mkdir -p $(CURDIR)/install/uwb_sim_3d/share/uwb_sim_3d/config/
	cp $(CURDIR)/config/anthor_3d.xml $(CURDIR)/install/uwb_sim_3d/share/uwb_sim_3d/config/anthor.xml

interfaces:
	colcon build --packages-select uwb_interfaces

location:
	colcon build --packages-select uwb_location

	mkdir -p $(CURDIR)/install/uwb_location/share/uwb_location/config/
	cp $(CURDIR)/config/anthor.xml $(CURDIR)/install/uwb_location/share/uwb_location/config/anthor.xml

location_3d:
	colcon build --packages-select uwb_location_3d

	mkdir -p $(CURDIR)/install/uwb_location_3d/share/uwb_location_3d/config/
	cp $(CURDIR)/config/anthor_3d.xml $(CURDIR)/install/uwb_location_3d/share/uwb_location_3d/config/anthor.xml

clean:
	rm -rf $(CURDIR)/build $(CURDIR)/install $(CURDIR)/log $(CURDIR)/launch/__pycache__