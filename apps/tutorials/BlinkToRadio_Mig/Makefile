COMPONENT=BlinkToRadioAppC

BUILD_EXTRA_DEPS = A.class
CLEAN_EXTRA = *.class *.java
#BlinkToRadio.class: $(wildcard *.java) BlinkToRadioMsg.java
#	javac -target 1.4 -source 1.4 *.java


# *.class   $.java, * could be different from $ or the same.
A.class: B.java
	javac B.java

#BlinkToRadioMsg.java:
#	mig java -target=null $(CFLAGS) -java-classname=BlinkToRadioMsg BlinkToRadio.h BlinkToRadioMsg -o $@
# the last BlinkToRadio should be consistent with the struct in header file.
# BlinkToRadioMsg.java:7: error: class BlinkToRadio is public, should be declared in a file named BlinkToRadio.java
# After modifying the first BlinkToRadioMsg to BlinkToRadio.

# ******	mig	Invoke mig
# ******	java	Build a Java class
# ******	-target=null	For the nullplatform
# ******	-java-classname=BlinkToRadio	Name the Java class BlinkToRadio
# ******	BlinkToRadio.h	The structure is in TestSerial.h
# ******	BlinkToRadioMsg	The structure is named TestSerialMsg, and this structure is indexed in module file.
# ******	-o $@	Write the file to $@, which is TestSerialMsg.java

B.java:
	mig java -target=null -java-classname=B BlinkToRadio.h BlinkToRadioMsg -o $@

 
include $(MAKERULES)

