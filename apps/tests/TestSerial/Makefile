COMPONENT=TestSerialAppC

# BUILD_EXTRA_DEPS tells the TinyOS make system that 
# the TinyOS application has additional dependencies 
# that must be satisfied before it can be built.
BUILD_EXTRA_DEPS += TestSerial.class

# CLEAN_EXTRA line tells the make system extra things 
# that need to be done when a user types make clean to clean up.
CLEAN_EXTRA = *.class TestSerialMsg.java


CFLAGS += -I$(TOSDIR)/lib/T2Hack


# tells that TestSerial.class depends on all of the .java files 
# in the directory as well as TestSerialMsg.java. 
# Once all of these dependencies are resolved, 
# the make system will call javac *.java, which creates TestSerial.class.

#************	Note that there must be a tab before javac, and not just spaces
TestSerial.class: $(wildcard *.java) TestSerialMsg.java
	javac -target 1.4 -source 1.4 *.java


# tells the make system how to create TestSerialMsg.java, 
# the Java class representing the packet sent between the mote and PC. 
# Because TestSerialMsg.java is a dependency for TestSerial.class, 
# make will create it if it is needed. To create TestSerialMsg.java, 
# the Makefile invokes the mig tool. 

#************	mig	Invoke mig
#************	java	Build a Java class
#************	-target=null	For the nullplatform
#************	-java-classname=TestSerialMsg	Name the Java class TestSerialMsg
#************	TestSerial.h	The structure is in TestSerial.h
#************	TestSerialMsg	The structure is named TestSerialMsg
#************	-o $@	Write the file to $@, which is TestSerialMsg.java
TestSerialMsg.java:
	mig java -target=null $(CFLAGS) -java-classname=TestSerialMsg TestSerial.h test_serial_msg -o $@


include $(MAKERULES)

