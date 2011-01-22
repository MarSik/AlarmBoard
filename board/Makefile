PROJECT = alarm
SCHEMAS = alarm.sch
PACKAGES = ./packages

.PRECIOUS: .drc .sch .partslist3 .pcb .new.pcb .net

all: drc pcb
drc: $(PROJECT).drc

pcb: $(SCHEMAS) $(PROJECT).drc
	gsch2pcb -d $(PACKAGES) -o $(PROJECT) --gnetlist partslist3 $(SCHEMAS)

$(PROJECT).drc: $(SCHEMAS)
	gnetlist -g drc2 -o $(PROJECT).drc $(SCHEMAS)
	cat $@
	#grep -q ERROR $(PROJECT).drc && exit 1

clean:
	rm -rf *.drc *.partslist*
