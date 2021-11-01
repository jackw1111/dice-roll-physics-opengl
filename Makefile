OBJ=obj/
SRC=src/
INCLUDE=include/
FLAGS =-Wall -Wextra -fPIC -Wno-unused-parameter -Wno-unused-function
DEBUG = -g -O0
RELEASE = -O3
BUILD = $(RELEASE)
GET_OBJS = $(wildcard $(INCLUDE)*.h)
GET_OBJS2 = $(GET_OBJS:.h=.o)
OBJS = $(subst include/, obj/, $(GET_OBJS2))
LIBS = -ldl -lassimp -lglfw

project: $(OBJS)
	@echo "building project..."
	@g++ $(BUILD) -I $(INCLUDE) $(OBJS) -o bin/demo  $(LIBS) 
	@echo "Finished."

$(OBJ)%.o : $(SRC)%.cpp
	@echo building $* ...
	@g++ $(BUILD) -c -I $(INCLUDE) $(FLAGS) $< -o $@ -lGL $(LIBS)

obj/glad.o : src/glad.c
	@echo building glad ...
	@gcc -c -I $(INCLUDE) $(FLAGS) $< -o $@ $(LIBS)


clean:
	@echo "cleaning project..."
	@rm bin/demo obj/*.o