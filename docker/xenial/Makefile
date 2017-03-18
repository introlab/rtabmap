all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build  - build images"
	@echo "   1. make pull   - pull images"
	@echo "   1. make clean  - remove images"
	@echo ""

build:
	@docker build --tag=rtabmap .

pull:
	@docker pull rtabmap

clean:
	@docker rmi -f rtabmap
