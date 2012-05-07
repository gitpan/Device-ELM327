package Device::ELM327;

use strict;
use warnings;

use Data::Dumper;

my $null = "\x0";
my $lf = "\xa";
my $cr = "\xd";

my $on = 1;
my $off = 0;

my $max_ports_to_search = 32;

my @has_sub_command = (0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0);

BEGIN
{
  if ($^O eq "MSWin32")
  {
    require Win32::SerialPort;
  }
  else
  {
    require Device::SerialPort;
    Device::SerialPort->import qw( :PARAM :STAT 0.07 );
  }
}

=head1 NAME

Device::ELM327 - Methods for reading OBD data with an ELM327 module.

=head1 VERSION

Version 0.04

=cut

our $VERSION = '0.04';

#*****************************************************************

=head1 SYNOPSIS

This module provides a Perl interface to a device containing an Elm Electronics ELM327 OBD Interpreter and provides access to the following functions:

 Read OBD parameters and extract individual values from results.
 Read OBD Trouble Codes and expand them to their full form.
 Reset OBD Trouble Codes.
 Read ELM327 parameters.
 Write and write the ELM327 data byte.
 Calibrate ELM327 Voltage.
 Switchable diagnostic trace and replay function for debugging.

The module is written entirely in Perl and works with both Linux and Windows. Depending on which operating system it is run on it uses either the Win32::SerialPort or Device::SerialPort module so it should work on any platform that supports one of them.

 use Device::ELM327;

  my $obd = Device::ELM327->new();

  # Read status information...
  $obd->Show("ELM identity");
  $obd->Show("Vehicle Identification Number");
  $obd->Show("Engine RPM");
  $obd->ShowTroubleCodes();

  undef $obd;


=head1 SUBROUTINES/METHODS


=head2 new - the constructor for the module.

To open the device and have it search for an ELM module:

 my $obd = Device::ELM327->new();

If you know the port name (e.g. 'COM5', '/dev/ttyUSB7' etc) it may be 
quicker to pass it into the function:

 my $obd = Device::ELM327->new($port_name);

If you want extra debugging information, it can be enabled by setting 
$debug_level to a positive number in the range 1 to 3, with more 
information being displayed as the number increases:

 my $obd = Device::ELM327->new($port_name, $debug_level);

A value of either undef or "" can be passed for the $port_name:

 my $obd = Device::ELM327->new("", $debug_level);

The module can replay previously captured debugging information:

 my $obd = Device_ELM327->new(undef, $debug_level, $replay_filename);

To produce a file containing replayable data simply set $debug_level to
1 or higher and pipe the output to a text file:

 perl test.pl>test_output.txt

=cut

sub new
{
	my ($class, $port_name, $debug_level, $replay_filename) = @_;
	my $self = bless { }, $class;

	$self->{'version'} = $VERSION;

	$self->{'debug_level'} = 0;
	if (defined($debug_level))
	{
	  $self->{'debug_level'} = $debug_level;
	}

  $self->{'ELM_type'} = "NONE";
  $self->{'bus_type'} = "unknown";
	$self->{'replay_file'} = 0;
	$self->{'replay_response'} = ();
 
	$self->{'last_command'} = 0;
	$self->{'last_sub_command'} = 0;
	$self->{'response'} = (); # Array of strings, one per line of the response.
	$self->{'response_length'} = 0; # Total number of characters in the response.
	$self->{'results'} = {};
	
	$self->{'number_of_results'} = 0;
	$self->{'command_addresses'} = [];
	$self->{'command_results'} = [];

	$self->{'trouble_codes'} = [];

	# ISO Standard Test Id's for use with function 6.
	$self->{'Standardized_Test_IDs'} = {
						"0" => {name => "ISO/SAE reserved"},
						"1" => {name => "Rich to lean sensor threshold voltage (constant)"},
						"2" => {name => "Lean to rich sensor threshold voltage (constant)"},
						"3" => {name => "Low sensor voltage for switch time calculation (constant)"},
						"4" => {name => "High sensor voltage for switch time calculation (constant)"},
						"5" => {name => "Rich to lean sensor switch time (calculated)"},
						"6" => {name => "Lean to rich sensor switch time (calculated)"},
						"7" => {name => "Minimum sensor voltage for test cycle (calculated)"},
						"8" => {name => "Maximum sensor voltage for test cycle (calculated)"},
						"9" => {name => "Time between sensor transitions (calculated)"},
						"10" => {name => "Sensor period (calculated)"},
						"11" => {name => "Exponential Weighted Moving Average misfire counts for last ten driving cycles"},
						"12" => {name => "Misfire counts for last/current driving cycles"},
						"13" => {name => "Reserved for future standardization"},
						};


	# ISO Unit and scaling identifiers
	$self->{'unit_and_scaling_identifiers'} = {
						"1" => {description => "Raw Value", modifier =>"+0", unit => ""},
						"2" => {description => "Raw Value", modifier =>"/10", unit => ""},
						"3" => {description => "Raw Value", modifier =>"/100", unit => ""},
						"4" => {description => "Raw Value", modifier =>"/100", unit => ""},
						"5" => {description => "Raw Value", modifier =>"*0.0000305", unit => ""},
						"6" => {description => "Raw Value", modifier =>"*0.000305", unit => ""},
						"7" => {description => "rotational frequency", modifier =>"/4", unit => "rpm"},
						"8" => {description => "Speed", modifier =>"/100", unit => "km/h"},
						"9" => {description => "Speed", modifier =>"+0", unit => "km/h"},
						"10" => {description => "Voltage", modifier =>"*0.000122", unit => "V"},
						"11" => {description => "Voltage", modifier =>"/1000", unit => "V"},
						"12" => {description => "Voltage", modifier =>"/100", unit => "V"},
						"13" => {description => "Current", modifier =>"*0.00390625", unit => "mA"},
						"14" => {description => "Current", modifier =>"*0.001", unit => "mA"},
						"15" => {description => "Current", modifier =>"*0.01", unit => "mA"},
						"16" => {description => "Time", modifier =>"+0", unit => "ms"},
						"17" => {description => "Time", modifier =>"/10", unit => "s"},
						"18" => {description => "Time", modifier =>"+0", unit => "s"},
						"19" => {description => "Resistance", modifier =>"/1000", unit => "Ohm"},
						"20" => {description => "Resistance", modifier =>"/1000", unit => "kOhm"},
						"21" => {description => "Resistance", modifier =>"+0", unit => "kOhm"},
						"22" => {description => "Temperature", modifier =>"/10-40", unit => "°C"},
						"23" => {description => "Pressure (Gauge)", modifier =>"*0.01", unit => "kPa"},
						"24" => {description => "Pressure (Air pressure)", modifier =>"*0.0117", unit => "kPa"},
						"25" => {description => "Pressure (Fuel pressure)", modifier =>"*0.079", unit => "kPa"},
						"26" => {description => "Pressure (Gauge)", modifier =>"+0", unit => "kPa"},
						"27" => {description => "Pressure (Diesel pressure)", modifier =>"*10", unit => "kPa"},
						"28" => {description => "Angle", modifier =>"*0.01", unit => "°"},
						"29" => {description => "Angle", modifier =>"/2", unit => "°"},
						"30" => {description => "Equivalence ratio (lambda)", modifier =>"*0.0000305", unit => "lambda"},
						"31" => {description => "Air/Fuel Ratio", modifier =>"*0.05", unit => "A/F ratio"},
						"32" => {description => "Ratio", modifier =>"*0.0039062", unit => ""},
						"33" => {description => "Frequency", modifier =>"/1000", unit => "Hz"},
						"34" => {description => "Frequency", modifier =>"+0", unit => "Hz"},
						"35" => {description => "Frequency", modifier =>"/1000", unit => "MHz"},
						"36" => {description => "Counts", modifier =>"+0", unit => "counts"},
						"37" => {description => "Distance", modifier =>"+0", unit => "km"},
						"38" => {description => "Voltage per time", modifier =>"/1000", unit => "V/ms"},
						"39" => {description => "Mass per time", modifier =>"/100", unit => "g/s"},
						"40" => {description => "Mass per time", modifier =>"+0", unit => "g/s"},
						"41" => {description => "Pressure per time", modifier =>"/1000", unit => "kPa/s"},
						"42" => {description => "Mass per time", modifier =>"/1000", unit => "kg/h"},
						"43" => {description => "Switches", modifier =>"+0", unit => "switches"},
						"44" => {description => "Mass per cylinder", modifier =>"/100", unit => "g/cyl"},
						"45" => {description => "Mass per stroke", modifier =>"/100", unit => "mg/stroke"},
						"46" => {description => "True/False", modifier =>"+0", unit => ""},
						"47" => {description => "Percent", modifier =>"/100", unit => "%"},
						"48" => {description => "Percent", modifier =>"*0.001526", unit => "%"},
						"49" => {description => "volume", modifier =>"/1000", unit => "L"},
						"50" => {description => "length", modifier =>"*0.0007747", unit => "mm"},
						"51" => {description => "Equivalence ratio (lambda)", modifier =>"*0.00024414", unit => "lambda"},
						"52" => {description => "Time", modifier =>"+0", unit => "min"},
						"53" => {description => "Time", modifier =>"/100", unit => "s"},
						"54" => {description => "Weight", modifier =>"/100", unit => "g"},
						"55" => {description => "Weight", modifier =>"/10", unit => "g"},
						"56" => {description => "Weight", modifier =>"+0", unit => "g"},
						"57" => {description => "Percent", modifier =>"/100", unit => "%"},

						"129" => {description => "Raw Value", modifier =>"+0", unit => ""},
						"130" => {description => "Raw Value", modifier =>"/10", unit => ""},
						"131" => {description => "Raw Value", modifier =>"/100", unit => ""},
						"132" => {description => "Raw Value", modifier =>"/1000", unit => ""},
						"133" => {description => "Raw Value", modifier =>"*0.0000305", unit => ""},
						"134" => {description => "Raw Value", modifier =>"*0.000305", unit => ""},
						
						"138" => {description => "Voltage", modifier =>"*0.000122", unit => "V"},
						"139" => {description => "Voltage", modifier =>"/1000", unit => "V"},
						"140" => {description => "Voltage", modifier =>"/100", unit => "V"},
						"141" => {description => "Current", modifier =>"*0.00390625", unit => "mA"},
						"142" => {description => "Current", modifier =>"*0.001", unit => "mA"},
						
						"144" => {description => "Time", modifier =>"/1000", unit => "s"},
						
						"150" => {description => "Temperature", modifier =>"/10", unit => "°C"},
						
						"156" => {description => "Angle", modifier =>"*0.01", unit => "°"},
						"157" => {description => "Angle", modifier =>"/2", unit => "°"},

						"168" => {description => "Mass per time", modifier =>"+0", unit => "g/s"},
						"169" => {description => "Pressure per time", modifier =>"/4", unit => "Pa/s"},

						"175" => {description => "Percent", modifier =>"/100", unit => "%"},
						"176" => {description => "Percent", modifier =>"*0.003052", unit => "%"},
						"177" => {description => "Voltage per time", modifier =>"*2", unit => "V/ms"},

						"253" => {description => "Pressure (absolute)", modifier =>"*0.001", unit => "kPa"},
						"254" => {description => "Pressure (vacuum)", modifier =>"/4", unit => "Pa"},

						};

  $self->{'get'} = {
            "ELM identity" => { command => "AT I", available => 1, result => [{type => "AT", modifier =>"", unit=>""}] },
            "Stored data byte" => { command => "AT RD", available => 1, result => [{type => "AT", modifier =>"", unit=>""}] },
            "Input Voltage" => { command => "AT RV", available => 1, result => [{type => "AT", modifier =>'=~ s/V//', unit=>"V"}] },
            "Ignition state" => { command => "AT IGN", available => 1, result => [{type => "AT", modifier =>"", unit=>""}] },

            "01 PIDs supported (01-20)" => { command => "01 00", available => 1,
            result => [
            {name => "Monitor status since DTCs cleared", type => "bool_0", modifier => "&128", unit => ""},
#            {name => "DTC that caused required freeze frame data storage", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Fuel systems status", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Calculated LOAD Value!", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Engine Coolant Temperature", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Short Term Fuel Trim - Bank 1/3", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Long Term Fuel Trim - Bank 1/3", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Short Term Fuel Trim - Bank 2/4", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Long Term Fuel Trim - Bank 2/4", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Fuel Rail Pressure (gauge)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Intake Manifold Absolute Pressure", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Engine RPM", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Vehicle Speed Sensor", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Ignition Timing Advance for #1 Cylinder", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Intake Air Temperature", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Air Flow Rate from Mass Air Flow Sensor", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Absolute Throttle Position", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Commanded Secondary Air Status", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Location of oxygen sensors 13", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Bank 1 - Sensor 1", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Bank 1 - Sensor 2", type => "bool_2", modifier => "&8", unit => ""},
            
            {name => "Bank 1 - Sensor 3", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Bank 1 - Sensor 4", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Bank 2 - Sensor 1 13", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Bank 2 - Sensor 2 13", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Bank 2 - Sensor 3", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Bank 2 - Sensor 4", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Bank 3 - Sensor 2", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Bank 4 - Sensor 1", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Bank 4 - Sensor 2", type => "bool_3", modifier => "&32", unit => ""},
            {name => "OBD requirements to which vehicle is designed", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Location of oxygen sensors 1D", type => "bool_3", modifier => "&8", unit => ""},
            {name => "Bank 2 - Sensor 1 1D", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Bank 2 - Sensor 2 1D", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Bank 3 - Sensor 1", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Auxiliary Input Status", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Time Since Engine Start", type => "bool_3", modifier => "&2", unit => ""},
            {name => "01 PIDs supported (21-40)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Monitor status since DTCs cleared" => { command => "01 01",
            result => [
            {name => "Number of DTCs stored in this ECU", type => "byte_0", modifier => "&127", unit => ""},
            {name => "Malfunction Indicator Lamp (MIL) Status", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Misfire monitoring supported", type => "bool_1", modifier => "&1", unit => ""},
            {name => "Fuel system monitoring supported", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Comprehensive component monitoring supported", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Misfire monitoring complete", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Fuel system monitoring complete", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Comprehensive component monitoring complete", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Catalyst monitoring supported", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Heated catalyst monitoring supported", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Evaporative system monitoring supported", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Secondary air system monitoring supported", type => "bool_2", modifier => "&8", unit => ""},
            {name => "A/C system refrigerant monitoring supported", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Oxygen sensor monitoring supported", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Oxygen sensor heater monitoring supported", type => "bool_2", modifier => "&64", unit => ""},
            {name => "EGR system monitoring supported", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Catalyst monitoring complete", type => "bool_3", modifier => "&1", unit => ""},
            {name => "Heated catalyst monitoring complete", type => "bool_3", modifier => "&2", unit => ""},
            {name => "Evaporative system monitoring complete", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Secondary air system monitoring complete", type => "bool_3", modifier => "&8", unit => ""},
            {name => "A/C system refrigerant monitoring complete", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Oxygen sensor monitoring complete", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Oxygen sensor heater monitoring complete", type => "bool_3", modifier => "&64", unit => ""},
            {name => "EGR system monitoring complete", type => "bool_3", modifier => "&128", unit => ""},
            ] },
            # DTC that caused required freeze frame data storage (PID 02) only exists for command 2.
            "Fuel systems status" => { command => "01 03",
            result => [
            {name => "Fuel system 1 status", type => "byte_0", modifier => "&31", unit => "",
            alternatives => [
            {value => 1, meaning => "Open loop - has not yet satisfied conditions to go closed loop"},
            {value => 2, meaning => "Closed loop - using oxygen sensor(s) as feedback for fuel control"},
            {value => 4, meaning => "Open loop due to driving conditions (e.g., power enrichment, deceleration enleanment)"},
            {value => 8, meaning => "Open loop - due to detected system fault"},
            {value => 16, meaning => "Closed loop, but fault with at least one oxygen sensor - may be using single oxygen sensor for fuel control"}
            ]
            },
            {name => "Fuel system 2 status", type => "byte_1", modifier => "&31", unit => "",
            alternatives => [
            {value => 1, meaning => "Open loop - has not yet satisfied conditions to go closed loop"},
            {value => 2, meaning => "Closed loop - using oxygen sensor(s) as feedback for fuel control"},
            {value => 4, meaning => "Open loop due to driving conditions (e.g., power enrichment, deceleration enleanment)"},
            {value => 8, meaning => "Open loop - due to detected system fault"},
            {value => 16, meaning => "Closed loop, but fault with at least one oxygen sensor - may be using single oxygen sensor for fuel control"}
            ]
            }
            ] },

            "Calculated LOAD Value!" => { command => "01 04", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Engine Coolant Temperature" => { command => "01 05", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },

            "Short Term Fuel Trim - Bank 1/3" => { command => "01 06",
            result => [
            {name => "Short Term Fuel Trim - Bank 1", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Short Term Fuel Trim - Bank 3", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Long Term Fuel Trim - Bank 1/3" => { command => "01 07",
            result => [
            {name => "Long Term Fuel Trim - Bank 1", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Long Term Fuel Trim - Bank 3", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Short Term Fuel Trim - Bank 2/4" => { command => "01 08",
            result => [
            {name => "Short Term Fuel Trim - Bank 2", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Short Term Fuel Trim - Bank 4", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Long Term Fuel Trim - Bank 2/4" => { command => "01 09",
            result => [
            {name => "Long Term Fuel Trim - Bank 2", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Long Term Fuel Trim - Bank 4", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            
            "Fuel Rail Pressure (gauge)" => { command => "01 0A", result => [{type => "byte_0", modifier => "*3", unit => "kPa"}] },
            "Intake Manifold Absolute Pressure" => { command => "01 0B", result => [{type => "byte_0", modifier => "*1", unit => "kPa"}] },
            "Engine RPM" => { command => "01 0C", result => [{type => "word_0", modifier => "/4", unit => "RPM"}] },
            "Vehicle Speed Sensor" => { command => "01 0D", result => [{type => "byte_0", modifier => "+0", unit => "km/h"}] },
            "Ignition Timing Advance for #1 Cylinder" => { command => "01 0E", result => [{type => "signed_byte_0", modifier => "/2", unit => "°"}] },
            "Intake Air Temperature" => { command => "01 0F", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },
            "Air Flow Rate from Mass Air Flow Sensor" => { command => "01 10", result => [{type => "word_0", modifier => "/100", unit => "g/s"}] },
            "Absolute Throttle Position" => { command => "01 11", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Commanded Secondary Air Status" => { command => "01 12", result => [{type => "byte_0", modifier => "*100/255", unit => "", 
            alternatives => [
            {value => 1, meaning => "upstream of first catalytic converter"},
            {value => 2, meaning => "downstream of first catalytic converter inlet"},
            {value => 4, meaning => "atmosphere / off"}
            ] }
            ] },

            "Location of oxygen sensors 13" => { command => "01 13",
            result => [
            {name => "Bank 1 - Sensor 1 present", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Bank 1 - Sensor 2 present", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Bank 1 - Sensor 3 present", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Bank 1 - Sensor 4 present", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Bank 2 - Sensor 1 present", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Bank 2 - Sensor 2 present", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Bank 2 - Sensor 3 present", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Bank 2 - Sensor 4 present", type => "bool_0", modifier => "&128", unit => ""}
            ] },

            "Bank 1 - Sensor 1" => { command => "01 14",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },
            
            "Bank 1 - Sensor 2" => { command => "01 15",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 1 - Sensor 3" => { command => "01 16",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 1 - Sensor 4" => { command => "01 17",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 1 13" => { command => "01 18",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 2 13" => { command => "01 19",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 3" => { command => "01 1A",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 4" => { command => "01 1B",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },


            "Bank 2 - Sensor 1 1D" => { command => "01 16",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 2 - Sensor 2 1D" => { command => "01 17",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },
            
            "Bank 3 - Sensor 1" => { command => "01 18",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 3 - Sensor 2" => { command => "01 19",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 4 - Sensor 1" => { command => "01 1A",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Bank 4 - Sensor 2" => { command => "01 1B",
            result => [
            {name => "Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },


            "OBD requirements to which vehicle is designed" => { command => "01 1C", result => [{type => "byte_0", modifier => "+0", unit => "",
            alternatives => [
            {value => 1, meaning => "OBD II (California ARB)"},
            {value => 2, meaning => "OBD (Federal EPA)"},
            {value => 3, meaning => "OBD and OBD II"},
            {value => 4, meaning => "OBD I"},
            {value => 5, meaning => "Not OBD compliant"},
            {value => 6, meaning => "EOBD"},
            {value => 7, meaning => "EOBD and OBD II"},
            {value => 8, meaning => "EOBD and OBD"},
            {value => 9, meaning => "EOBD, OBD and OBD II"},
            {value => 10, meaning => "JOBD"},
            {value => 11, meaning => "JOBD and OBD II"},
            {value => 12, meaning => "JOBD and EOBD"},
            {value => 13, meaning => "JOBD, EOBD, and OBD II"}
            ] }
            ] },
            
            "Location of oxygen sensors 1D" => { command => "01 1D",
            result => [
            {name => "Bank 1 - Sensor 1 present", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Bank 1 - Sensor 2 present", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Bank 2 - Sensor 1 present", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Bank 2 - Sensor 2 present", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Bank 3 - Sensor 1 present", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Bank 3 - Sensor 2 present", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Bank 4 - Sensor 1 present", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Bank 4 - Sensor 2 present", type => "bool_0", modifier => "&128", unit => ""},
            ] },
            
            "Auxiliary Input Status" => { command => "01 1E", result => [{name => "Power Take Off (PTO) Status", type => "bool_0", modifier => "&1", unit => ""}] },
            "Time Since Engine Start" => { command => "01 1F", result => [{type => "word_0", modifier => "*10", unit => "s"}] },

            "01 PIDs supported (21-40)" => { command => "01 20",
            result => [
            {name => "Distance Travelled While MIL is Activated", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Fuel Rail Pressure relative to manifold vacuum", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Fuel Rail Pressure", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Bank 1 - Sensor 1 (wide range O2S)", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Bank 1 - Sensor 2 (wide range O2S)", type => "bool_0", modifier => "&8", unit => ""},
            
            {name => "Bank 1 - Sensor 3 (wide range O2S)", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Bank 1 - Sensor 4 (wide range O2S)", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Bank 2 - Sensor 1 (wide range O2S) 13", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Bank 2 - Sensor 1 (wide range O2S) 1D", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Bank 2 - Sensor 2 (wide range O2S) 1D", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Bank 3 - Sensor 1 (wide range O2S)", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Bank 2 - Sensor 2 (wide range O2S) 13", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Bank 2 - Sensor 3 (wide range O2S)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Bank 2 - Sensor 4 (wide range O2S)", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Bank 3 - Sensor 2 (wide range O2S)", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Bank 4 - Sensor 1 (wide range O2S)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Bank 4 - Sensor 2 (wide range O2S)", type => "bool_1", modifier => "&32", unit => ""},
            
            {name => "Commanded EGR", type => "bool_1", modifier => "&16", unit => ""},
            {name => "EGR Error", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Commanded Evaporative Purge", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Fuel Level Input", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Number of warm-ups since diagnostic trouble codes cleared", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Distance since diagnostic trouble codes cleared", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Evap System Vapor Pressure", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Barometric Pressure", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Bank 1 - Sensor 1 (wide range O2S) current", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Bank 1 - Sensor 2 (wide range O2S) current", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Bank 1 - Sensor 3 (wide range O2S) current", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Bank 1 - Sensor 4 (wide range O2S) current", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Bank 2 - Sensor 1 (wide range O2S) current 13", type => "bool_2", modifier => "&1", unit => ""},

            {name => "Bank 2 - Sensor 1 (wide range O2S) current 1D", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Bank 2 - Sensor 2 (wide range O2S) current 1D", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Bank 3 - Sensor 1 (wide range O2S) current", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Bank 2 - Sensor 2 (wide range O2S) current 13", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Bank 2 - Sensor 3 (wide range O2S) current", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Bank 2 - Sensor 4 (wide range O2S) current", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Bank 3 - Sensor 2 (wide range O2S) current", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Bank 4 - Sensor 1 (wide range O2S) current", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Bank 4 - Sensor 2 (wide range O2S) current", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Catalyst Temperature Bank 1, Sensor 1", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Catalyst Temperature Bank 1, Sensor 2", type => "bool_3", modifier => "&8", unit => ""},
            {name => "Catalyst Temperature Bank 2, Sensor 1", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Catalyst Temperature Bank 2, Sensor 2", type => "bool_3", modifier => "&2", unit => ""},
            {name => "01 PIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Distance Travelled While MIL is Activated" => { command => "01 21", result => [{type => "word_0", modifier => "+0", unit => "km"}] },
            "Fuel Rail Pressure relative to manifold vacuum" => { command => "01 22", result => [{type => "word_0", modifier => "*0.079", unit => "kPa"}] },
            "Fuel Rail Pressure" => { command => "01 23", result => [{type => "word_0", modifier => "*10", unit => "kPa"}] },

            "Bank 1 - Sensor 1 (wide range O2S)" => { command => "01 24",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },
            
            "Bank 1 - Sensor 2 (wide range O2S)" => { command => "01 25",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 1 - Sensor 3 (wide range O2S)" => { command => "01 26",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 1 - Sensor 4 (wide range O2S)" => { command => "01 27",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 1 (wide range O2S) 13" => { command => "01 28",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 2 (wide range O2S) 13" => { command => "01 29",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 3 (wide range O2S)" => { command => "01 2A",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 4 (wide range O2S)" => { command => "01 2B",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },


            "Bank 2 - Sensor 1 (wide range O2S) 1D" => { command => "01 26",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 2 - Sensor 2 (wide range O2S) 1D" => { command => "01 27",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 3 - Sensor 1 (wide range O2S)" => { command => "01 28",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 3 - Sensor 2 (wide range O2S)" => { command => "01 29",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 4 - Sensor 1 (wide range O2S)" => { command => "01 2A",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Bank 4 - Sensor 2 (wide range O2S)" => { command => "01 2B",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Commanded EGR" => { command => "01 2C", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "EGR Error" => { command => "01 2D", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Commanded Evaporative Purge" => { command => "01 2E", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Fuel Level Input" => { command => "01 2F", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Number of warm-ups since diagnostic trouble codes cleared" => { command => "01 30", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Distance since diagnostic trouble codes cleared" => { command => "01 31", result => [{type => "word_0", modifier => "+0", unit => "km"}] },
            "Evap System Vapor Pressure" => { command => "01 32", result => [{type => "signed_word_0", modifier => "*0.25", unit => "Pa"}] },
            "Barometric Pressure" => { command => "01 33", result => [{type => "byte_0", modifier => "+0", unit => "kPa"}] },

            "Bank 1 - Sensor 1 (wide range O2S) current" => { command => "01 34",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },
            
            "Bank 1 - Sensor 2 (wide range O2S) current" => { command => "01 35",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 1 - Sensor 3 (wide range O2S) current" => { command => "01 36",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 1 - Sensor 4 (wide range O2S) current" => { command => "01 37",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 1 (wide range O2S) current 13" => { command => "01 38",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 2 (wide range O2S) current 13" => { command => "01 39",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 3 (wide range O2S) current" => { command => "01 3A",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 4 (wide range O2S) current" => { command => "01 3B",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },


            "Bank 2 - Sensor 1 (wide range O2S) current 1D" => { command => "01 36",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 2 - Sensor 2 (wide range O2S) current 1D" => { command => "01 37",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 3 - Sensor 1 (wide range O2S) current" => { command => "01 38",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 3 - Sensor 2 (wide range O2S) current" => { command => "01 39",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 4 - Sensor 1 (wide range O2S) current" => { command => "01 3A",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Bank 4 - Sensor 2 (wide range O2S) current" => { command => "01 3B",
            result => [
            {name => "Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },
            
            "Catalyst Temperature Bank 1, Sensor 1" => { command => "01 3C", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Catalyst Temperature Bank 1, Sensor 2" => { command => "01 3D", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Catalyst Temperature Bank 2, Sensor 1" => { command => "01 3E", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Catalyst Temperature Bank 2, Sensor 2" => { command => "01 3F", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },

            "01 PIDs supported (41-60)" => { command => "01 40",
            result => [
            {name => "Monitor status this driving cycle", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Control Module Voltage", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Absolute Load Value", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Commanded Equivalence Ratio", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Relative Throttle Position", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Ambiant Air Temperature", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Absolute Throttle Position B", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Absolute Throttle Position C", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Absolute Throttle Position D", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Absolute Throttle Position E", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Absolute Throttle Position F", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Commanded Throttle Actuator Control", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Minutes run by the engine while MIL activated", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Time since diagnostic trouble codes cleared", type => "bool_1", modifier => "&4", unit => ""},
            {name => "External Test Equipment Configuration Information #1", type => "bool_1", modifier => "&2", unit => ""},
            {name => "External Test Equipment Configuration Information #2", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Type of fuel currently being utilized by the vehicle", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Alcohol Fuel Percentage", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Absolute Evap System Vapour Pressure", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Evap System Vapour Pressure", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Short Term Secondary O2 Sensor Fuel Trim - Bank 1", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Long Term Secondary O2 Sensor Fuel Trim - Bank 1", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Short Term Secondary O2 Sensor Fuel Trim - Bank 2", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Long Term Secondary O2 Sensor Fuel Trim - Bank 2", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Short Term Secondary O2 Sensor Fuel Trim - Bank 3", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Long Term Secondary O2 Sensor Fuel Trim - Bank 3", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Short Term Secondary O2 Sensor Fuel Trim - Bank 4", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Long Term Secondary O2 Sensor Fuel Trim - Bank 4", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Fuel Rail Pressure (absolute)", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Relative Accelerator Pedal Position", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "01 PIDs supported (61-80)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            
            "Monitor status this driving cycle" => { command => "01 41",
            result => [
            {name => "Misfire monitoring Enable Status", type => "bool_1", modifier => "&1", unit => ""},
            {name => "Fuel system monitoring Enable Status", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Comprehensive component monitoring Enable Status", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Misfire monitoring Completion Status", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Fuel system monitoring Completion Status", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Comprehensive component monitoring Completion Status", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Catalyst monitoring Enable Status", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Heated catalyst monitoring Enable Status", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Evaporative system monitoring Enable Status", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Secondary air system monitoring Enable Status", type => "bool_2", modifier => "&8", unit => ""},
            {name => "A/C system refrigerant monitoring Enable Status", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Oxygen sensor monitoring Enable Status", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Oxygen sensor heater monitoring Enable Status", type => "bool_2", modifier => "&64", unit => ""},
            {name => "EGR monitoring Enable Status", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Catalyst monitoring Completion Status", type => "bool_3", modifier => "&1", unit => ""},
            {name => "Heated catalyst monitoring Completion Status", type => "bool_3", modifier => "&2", unit => ""},
            {name => "Evaporative system monitoring Completion Status", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Secondary air system monitoring Completion Status", type => "bool_3", modifier => "&8", unit => ""},
            {name => "A/C system refrigerant monitoring Completion Status", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Oxygen sensor monitoring Completion Status", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Oxygen sensor heater monitoring Completion Status", type => "bool_3", modifier => "&64", unit => ""},
            {name => "EGR monitoring Completion Status", type => "bool_3", modifier => "&128", unit => ""}
            ] },
            
            "Control Module Voltage" => { command => "01 42", result => [{type => "word_0", modifier => "*0.001", unit => "V"}] },
            "Absolute Load Value" => { command => "01 43", result => [{type => "word_0", modifier => "*100/255", unit => "%"}] },
            "Commanded Equivalence Ratio" => { command => "01 44", result => [{type => "word_0", modifier => "*0.0000305", unit => ""}] },
            "Relative Throttle Position" => { command => "01 45", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Ambiant Air Temperature" => { command => "01 46", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },
            "Absolute Throttle Position B" => { command => "01 47", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Absolute Throttle Position C" => { command => "01 48", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Accelerator Pedal Position D" => { command => "01 49", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Accelerator Pedal Position E" => { command => "01 4A", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Accelerator Pedal Position F" => { command => "01 4B", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Commanded Throttle Actuator Control" => { command => "01 4C", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Minutes run by the engine while MIL activated" => { command => "01 4D", result => [{type => "word_0", modifier => "+0", unit => "min"}] },
            "Time since diagnostic trouble codes cleared" => { command => "01 4E", result => [{type => "word_0", modifier => "+0", unit => "min"}] },
            "External Test Equipment Configuration Information #1" => { command => "01 4F",
            result => [
            {name => "Maximum value for Equivalence Ratio", type => "byte_0", modifier => "+0", unit => ""},
            {name => "Maximum value for Oxygen Sensor Voltage", type => "byte_1", modifier => "+0", unit => "V"},
            {name => "Maximum value for Oxygen Sensor Current", type => "byte_2", modifier => "+0", unit => "mA"},
            {name => "Maximum value for Intake Manifold Absolute Pressure", type => "byte_3", modifier => "+0", unit => "kPa"},
            ] },
            "External Test Equipment Configuration Information #2" => { command => "01 50",
            result => [
            {name => "Maximum value for Air Flow Rate from Mass Air Flow Sensor", type => "byte_0", modifier => "+0", unit => "g/s"}
            ] },

            "Type of fuel currently being utilized by the vehicle" => { command => "01 51", result => [{type => "byte_0", modifier => "+0", unit => "",
            alternatives => [
            {value => 1, meaning => "Gasoline/petrol"},
            {value => 2, meaning => "Methanol"},
            {value => 3, meaning => "Ethanol"},
            {value => 4, meaning => "Diesel"},
            {value => 5, meaning => "Liquefied Petroleum Gas (LPG)"},
            {value => 6, meaning => "Compressed Natural Gas (CNG)"},
            {value => 7, meaning => "Propane"},
            {value => 8, meaning => "Battery/electric"},
            {value => 9, meaning => "Bi-fuel vehicle using gasoline"},
            {value => 10, meaning => "Bi-fuel vehicle using methanol"},
            {value => 11, meaning => "Bi-fuel vehicle using ethanol"},
            {value => 12, meaning => "Bi-fuel vehicle using LPG"},
            {value => 13, meaning => "Bi-fuel vehicle using CNG"},
            {value => 14, meaning => "Bi-fuel vehicle using propane"},
            {value => 15, meaning => "Bi-fuel vehicle using battery"}
            ] }
            ] },
            
            "Alcohol Fuel Percentage" => { command => "01 52", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Absolute Evap System Vapour Pressure" => { command => "01 53", result => [{type => "word_0", modifier => "/200", unit => "kPa"}] },
            "Evap System Vapour Pressure" => { command => "01 54", result => [{type => "signed_word_0", modifier => "+1", unit => "Pa"}] },
            "Short Term Secondary O2 Sensor Fuel Trim - Bank 1" => { command => "01 55", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Long Term Secondary O2 Sensor Fuel Trim - Bank 1" => { command => "01 56", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Short Term Secondary O2 Sensor Fuel Trim - Bank 2" => { command => "01 57", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Long Term Secondary O2 Sensor Fuel Trim - Bank 2" => { command => "01 58", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },

            "Short Term Secondary O2 Sensor Fuel Trim - Bank 3" => { command => "01 55", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Long Term Secondary O2 Sensor Fuel Trim - Bank 3" => { command => "01 56", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Short Term Secondary O2 Sensor Fuel Trim - Bank 4" => { command => "01 57", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Long Term Secondary O2 Sensor Fuel Trim - Bank 4" => { command => "01 58", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },

            "Fuel Rail Pressure (absolute)" => { command => "01 59", result => [{type => "word_0", modifier => "*10", unit => "kPa"}] },
            "Relative Accelerator Pedal Position" => { command => "01 5A", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },


            
            "02 PIDs supported (01-20)" => { command => "02 00", available => 1,
            result => [
            {name => "Freeze frame Monitor status since DTCs cleared", type => "bool_0", modifier => "&128", unit => ""},
            {name => "DTC that caused required freeze frame data storage", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Freeze frame Fuel systems status", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Freeze frame Calculated LOAD Value!", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Freeze frame Engine Coolant Temperature", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Freeze frame Short Term Fuel Trim - Bank 1/3", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Freeze frame Long Term Fuel Trim - Bank 1/3", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Freeze frame Short Term Fuel Trim - Bank 2/4", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Freeze frame Long Term Fuel Trim - Bank 2/4", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Freeze frame Fuel Rail Pressure (gauge)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Freeze frame Intake Manifold Absolute Pressure", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Freeze frame Engine RPM", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Freeze frame Vehicle Speed Sensor", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Freeze frame Ignition Timing Advance for #1 Cylinder", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Freeze frame Intake Air Temperature", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Freeze frame Air Flow Rate from Mass Air Flow Sensor", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Freeze frame Absolute Throttle Position", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Freeze frame Commanded Secondary Air Status", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Freeze frame Location of oxygen sensors 13", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 1", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 2", type => "bool_2", modifier => "&8", unit => ""},
            
            {name => "Freeze frame Bank 1 - Sensor 3", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 4", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 1 13", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Freeze frame Bank 2 - Sensor 2 13", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 3", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 4", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Freeze frame Bank 3 - Sensor 2", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Freeze frame Bank 4 - Sensor 1", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Freeze frame Bank 4 - Sensor 2", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Freeze frame OBD requirements to which vehicle is designed", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Freeze frame Location of oxygen sensors 1D", type => "bool_3", modifier => "&8", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 1 1D", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 2 1D", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Freeze frame Bank 3 - Sensor 1", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Freeze frame Auxiliary Input Status", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Freeze frame Time Since Engine Start", type => "bool_3", modifier => "&2", unit => ""},
            {name => "02 PIDs supported (21-40)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Freeze frame Monitor status since DTCs cleared" => { command => "02 01",
            result => [
            {name => "Freeze frame Number of DTCs stored in this ECU", type => "byte_0", modifier => "&127", unit => ""},
            {name => "Freeze frame Malfunction Indicator Lamp (MIL) Status", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Freeze frame Misfire monitoring supported", type => "bool_1", modifier => "&1", unit => ""},
            {name => "Freeze frame Fuel system monitoring supported", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Freeze frame Comprehensive component monitoring supported", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Freeze frame Misfire monitoring complete", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Freeze frame Fuel system monitoring complete", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Freeze frame Comprehensive component monitoring complete", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Freeze frame Catalyst monitoring supported", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Freeze frame Heated catalyst monitoring supported", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Freeze frame Evaporative system monitoring supported", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Freeze frame Secondary air system monitoring supported", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Freeze frame A/C system refrigerant monitoring supported", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Freeze frame Oxygen sensor monitoring supported", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Freeze frame Oxygen sensor heater monitoring supported", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Freeze frame EGR system monitoring supported", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Freeze frame Catalyst monitoring complete", type => "bool_3", modifier => "&1", unit => ""},
            {name => "Freeze frame Heated catalyst monitoring complete", type => "bool_3", modifier => "&2", unit => ""},
            {name => "Freeze frame Evaporative system monitoring complete", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Freeze frame Secondary air system monitoring complete", type => "bool_3", modifier => "&8", unit => ""},
            {name => "Freeze frame A/C system refrigerant monitoring complete", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Freeze frame Oxygen sensor monitoring complete", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Freeze frame Oxygen sensor heater monitoring complete", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Freeze frame EGR system monitoring complete", type => "bool_3", modifier => "&128", unit => ""},
            ] },

            "DTC that caused required freeze frame data storage" => { command => "02 02", result => [{type => "word_0", modifier => "+0", unit => ""}] },

            "Freeze frame Fuel systems status" => { command => "02 03",
            result => [
            {name => "Freeze frame Fuel system 1 status", type => "byte_0", modifier => "&31", unit => "",
            alternatives => [
            {value => 1, meaning => "Open loop - has not yet satisfied conditions to go closed loop"},
            {value => 2, meaning => "Closed loop - using oxygen sensor(s) as feedback for fuel control"},
            {value => 4, meaning => "Open loop due to driving conditions (e.g., power enrichment, deceleration enleanment)"},
            {value => 8, meaning => "Open loop - due to detected system fault"},
            {value => 16, meaning => "Closed loop, but fault with at least one oxygen sensor - may be using single oxygen sensor for fuel control"}
            ]
            },
            {name => "Freeze frame Fuel system 2 status", type => "byte_1", modifier => "&31", unit => "",
            alternatives => [
            {value => 1, meaning => "Open loop - has not yet satisfied conditions to go closed loop"},
            {value => 2, meaning => "Closed loop - using oxygen sensor(s) as feedback for fuel control"},
            {value => 4, meaning => "Open loop due to driving conditions (e.g., power enrichment, deceleration enleanment)"},
            {value => 8, meaning => "Open loop - due to detected system fault"},
            {value => 16, meaning => "Closed loop, but fault with at least one oxygen sensor - may be using single oxygen sensor for fuel control"}
            ]
            }
            ] },

            "Freeze frame Calculated LOAD Value!" => { command => "02 04", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Engine Coolant Temperature" => { command => "02 05", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },

            "Freeze frame Short Term Fuel Trim - Bank 1/3" => { command => "02 06",
            result => [
            {name => "Freeze frame Short Term Fuel Trim - Bank 1", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Freeze frame Short Term Fuel Trim - Bank 3", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Freeze frame Long Term Fuel Trim - Bank 1/3" => { command => "02 07",
            result => [
            {name => "Freeze frame Long Term Fuel Trim - Bank 1", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Freeze frame Long Term Fuel Trim - Bank 3", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Freeze frame Short Term Fuel Trim - Bank 2/4" => { command => "02 08",
            result => [
            {name => "Freeze frame Short Term Fuel Trim - Bank 2", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Freeze frame Short Term Fuel Trim - Bank 4", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            "Freeze frame Long Term Fuel Trim - Bank 2/4" => { command => "02 09",
            result => [
            {name => "Freeze frame Long Term Fuel Trim - Bank 2", type => "signed_byte_0", modifier => "*100/128", unit => "%"},
            {name => "Freeze frame Long Term Fuel Trim - Bank 4", type => "signed_byte_1", modifier => "*100/128", unit => "%"},
            ] },

            
            "Freeze frame Fuel Rail Pressure (gauge)" => { command => "02 0A", result => [{type => "byte_0", modifier => "*3", unit => "kPa"}] },
            "Freeze frame Intake Manifold Absolute Pressure" => { command => "02 0B", result => [{type => "byte_0", modifier => "*1", unit => "kPa"}] },
            "Freeze frame Engine RPM" => { command => "02 0C", result => [{type => "word_0", modifier => "/4", unit => "RPM"}] },
            "Freeze frame Vehicle Speed Sensor" => { command => "02 0D", result => [{type => "byte_0", modifier => "+0", unit => "km/h"}] },
            "Freeze frame Ignition Timing Advance for #1 Cylinder" => { command => "02 0E", result => [{type => "signed_byte_0", modifier => "/2", unit => "°"}] },
            "Freeze frame Intake Air Temperature" => { command => "02 0F", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },
            "Freeze frame Air Flow Rate from Mass Air Flow Sensor" => { command => "02 10", result => [{type => "word_0", modifier => "/100", unit => "g/s"}] },
            "Freeze frame Absolute Throttle Position" => { command => "02 11", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Commanded Secondary Air Status" => { command => "02 12", result => [{type => "byte_0", modifier => "*100/255", unit => "", 
            alternatives => [
            {value => 1, meaning => "upstream of first catalytic converter"},
            {value => 2, meaning => "downstream of first catalytic converter inlet"},
            {value => 4, meaning => "atmosphere / off"}
            ] }
            ] },

            "Freeze frame Location of oxygen sensors 13" => { command => "02 13",
            result => [
            {name => "Freeze frame Bank 1 - Sensor 1 present", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 2 present", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 3 present", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 4 present", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 1 present", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 2 present", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 3 present", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 4 present", type => "bool_0", modifier => "&128", unit => ""}
            ] },

            "Freeze frame Bank 1 - Sensor 1" => { command => "02 14",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },
            
            "Freeze frame Bank 1 - Sensor 2" => { command => "02 15",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 1 - Sensor 3" => { command => "02 16",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 1 - Sensor 4" => { command => "02 17",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 2 - Sensor 1 13" => { command => "02 18",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 2 - Sensor 2 13" => { command => "02 19",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 2 - Sensor 3" => { command => "02 1A",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 2 - Sensor 4" => { command => "02 1B",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },


            "Freeze frame Bank 2 - Sensor 1 1D" => { command => "02 16",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 2 - Sensor 2 1D" => { command => "02 17",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },
            
            "Freeze frame Bank 3 - Sensor 1" => { command => "02 18",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 3 - Sensor 2" => { command => "02 19",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 4 - Sensor 1" => { command => "02 1A",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },

            "Freeze frame Bank 4 - Sensor 2" => { command => "02 1B",
            result => [
            {name => "Freeze frame Oxygen Sensor Output Voltage (Bx-Sy)", type => "byte_0", modifier => "*0.005", unit => "V"},
            {name => "Freeze frame Short Term Fuel Trim (Bx-Sy)", type => "byte_1", modifier => "*100/255", unit => "%"}
            ] },


            "Freeze frame OBD requirements to which vehicle is designed" => { command => "02 1C", result => [{type => "byte_0", modifier => "+0", unit => "",
            alternatives => [
            {value => 1, meaning => "OBD II (California ARB)"},
            {value => 2, meaning => "OBD (Federal EPA)"},
            {value => 3, meaning => "OBD and OBD II"},
            {value => 4, meaning => "OBD I"},
            {value => 5, meaning => "Not OBD compliant"},
            {value => 6, meaning => "EOBD"},
            {value => 7, meaning => "EOBD and OBD II"},
            {value => 8, meaning => "EOBD and OBD"},
            {value => 9, meaning => "EOBD, OBD and OBD II"},
            {value => 10, meaning => "JOBD"},
            {value => 11, meaning => "JOBD and OBD II"},
            {value => 12, meaning => "JOBD and EOBD"},
            {value => 13, meaning => "JOBD, EOBD, and OBD II"}
            ] }
            ] },
            
            "Freeze frame Location of oxygen sensors 1D" => { command => "02 1D",
            result => [
            {name => "Freeze frame Bank 1 - Sensor 1 present", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 2 present", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 1 present", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 2 present", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Freeze frame Bank 3 - Sensor 1 present", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Freeze frame Bank 3 - Sensor 2 present", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Freeze frame Bank 4 - Sensor 1 present", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Freeze frame Bank 4 - Sensor 2 present", type => "bool_0", modifier => "&128", unit => ""},
            ] },
            
            "Freeze frame Auxiliary Input Status" => { command => "02 1E", result => [{name => "Freeze frame Power Take Off (PTO) Status", type => "bool_0", modifier => "&1", unit => ""}] },
            "Freeze frame Time Since Engine Start" => { command => "02 1F", result => [{type => "word_0", modifier => "*10", unit => "s"}] },

            "02 PIDs supported (21-40)" => { command => "02 20",
            result => [
            {name => "Freeze frame Distance Travelled While MIL is Activated", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Freeze frame Fuel Rail Pressure relative to manifold vacuum", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Freeze frame Fuel Rail Pressure", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 1 (wide range O2S)", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 2 (wide range O2S)", type => "bool_0", modifier => "&8", unit => ""},
            
            {name => "Freeze frame Bank 1 - Sensor 3 (wide range O2S)", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 4 (wide range O2S)", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 1 (wide range O2S) 13", type => "bool_0", modifier => "&1", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 1 (wide range O2S) 1D", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 2 (wide range O2S) 1D", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Freeze frame Bank 3 - Sensor 1 (wide range O2S)", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Freeze frame Bank 2 - Sensor 2 (wide range O2S) 13", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 3 (wide range O2S)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 4 (wide range O2S)", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Freeze frame Bank 3 - Sensor 2 (wide range O2S)", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Freeze frame Bank 4 - Sensor 1 (wide range O2S)", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Freeze frame Bank 4 - Sensor 2 (wide range O2S)", type => "bool_1", modifier => "&32", unit => ""},
            
            {name => "Freeze frame Commanded EGR", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Freeze frame EGR Error", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Freeze frame Commanded Evaporative Purge", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Freeze frame Fuel Level Input", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Freeze frame Number of warm-ups since diagnostic trouble codes cleared", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Freeze frame Distance since diagnostic trouble codes cleared", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Freeze frame Evap System Vapor Pressure", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Freeze frame Barometric Pressure", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 1 (wide range O2S) current", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 2 (wide range O2S) current", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 3 (wide range O2S) current", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Freeze frame Bank 1 - Sensor 4 (wide range O2S) current", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 1 (wide range O2S) current 13", type => "bool_2", modifier => "&1", unit => ""},

            {name => "Freeze frame Bank 2 - Sensor 1 (wide range O2S) current 1D", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 2 (wide range O2S) current 1D", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Freeze frame Bank 3 - Sensor 1 (wide range O2S) current", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Freeze frame Bank 2 - Sensor 2 (wide range O2S) current 13", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 3 (wide range O2S) current", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Freeze frame Bank 2 - Sensor 4 (wide range O2S) current", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Freeze frame Bank 3 - Sensor 2 (wide range O2S) current", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Freeze frame Bank 4 - Sensor 1 (wide range O2S) current", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Freeze frame Bank 4 - Sensor 2 (wide range O2S) current", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Freeze frame Catalyst Temperature Bank 1, Sensor 1", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Freeze frame Catalyst Temperature Bank 1, Sensor 2", type => "bool_3", modifier => "&8", unit => ""},
            {name => "Freeze frame Catalyst Temperature Bank 2, Sensor 1", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Freeze frame Catalyst Temperature Bank 2, Sensor 2", type => "bool_3", modifier => "&2", unit => ""},
            {name => "02 PIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Freeze frame Distance Travelled While MIL is Activated" => { command => "02 21", result => [{type => "word_0", modifier => "+0", unit => "km"}] },
            "Freeze frame Fuel Rail Pressure relative to manifold vacuum" => { command => "02 22", result => [{type => "word_0", modifier => "*0.079", unit => "kPa"}] },
            "Freeze frame Fuel Rail Pressure" => { command => "02 23", result => [{type => "word_0", modifier => "*10", unit => "kPa"}] },

            "Freeze frame Bank 1 - Sensor 1 (wide range O2S)" => { command => "02 24",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },
            
            "Freeze frame Bank 1 - Sensor 2 (wide range O2S)" => { command => "02 25",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 1 - Sensor 3 (wide range O2S)" => { command => "02 26",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 1 - Sensor 4 (wide range O2S)" => { command => "02 27",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 2 - Sensor 1 (wide range O2S) 13" => { command => "02 28",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 2 - Sensor 2 (wide range O2S) 13" => { command => "02 29",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 2 - Sensor 3 (wide range O2S)" => { command => "02 2A",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 2 - Sensor 4 (wide range O2S)" => { command => "02 2B",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },


            "Freeze frame Bank 2 - Sensor 1 (wide range O2S) 1D" => { command => "02 26",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 2 - Sensor 2 (wide range O2S) 1D" => { command => "02 27",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 3 - Sensor 1 (wide range O2S)" => { command => "02 28",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 3 - Sensor 2 (wide range O2S)" => { command => "02 29",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 4 - Sensor 1 (wide range O2S)" => { command => "02 2A",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Bank 4 - Sensor 2 (wide range O2S)" => { command => "02 2B",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Voltage (Bx-Sy)", type => "word_1", modifier => "*0.000122", unit => "V"}
            ] },

            "Freeze frame Commanded EGR" => { command => "02 2C", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame EGR Error" => { command => "02 2D", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Freeze frame Commanded Evaporative Purge" => { command => "02 2E", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Fuel Level Input" => { command => "02 2F", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Number of warm-ups since diagnostic trouble codes cleared" => { command => "02 30", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Freeze frame Distance since diagnostic trouble codes cleared" => { command => "02 31", result => [{type => "word_0", modifier => "+0", unit => "km"}] },
            "Freeze frame Evap System Vapor Pressure" => { command => "02 32", result => [{type => "signed_word_0", modifier => "*0.25", unit => "Pa"}] },
            "Freeze frame Barometric Pressure" => { command => "02 33", result => [{type => "byte_0", modifier => "+0", unit => "kPa"}] },

            "Freeze frame Bank 1 - Sensor 1 (wide range O2S) current" => { command => "02 34",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },
            
            "Freeze frame Bank 1 - Sensor 2 (wide range O2S) current" => { command => "02 35",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 1 - Sensor 3 (wide range O2S) current" => { command => "02 36",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 1 - Sensor 4 (wide range O2S) current" => { command => "02 37",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 2 - Sensor 1 (wide range O2S) current 13" => { command => "02 38",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 2 - Sensor 2 (wide range O2S) current 13" => { command => "02 39",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 2 - Sensor 3 (wide range O2S) current" => { command => "02 3A",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 2 - Sensor 4 (wide range O2S) current" => { command => "02 3B",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },


            "Freeze frame Bank 2 - Sensor 1 (wide range O2S) current 1D" => { command => "02 36",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 2 - Sensor 2 (wide range O2S) current 1D" => { command => "02 37",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 3 - Sensor 1 (wide range O2S) current" => { command => "02 38",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 3 - Sensor 2 (wide range O2S) current" => { command => "02 39",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 4 - Sensor 1 (wide range O2S) current" => { command => "02 3A",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },

            "Freeze frame Bank 4 - Sensor 2 (wide range O2S) current" => { command => "02 3B",
            result => [
            {name => "Freeze frame Equivalence Ratio (lambda) (Bx-Sy)", type => "word_0", modifier => "*0.0000305", unit => ""},
            {name => "Freeze frame Oxygen Sensor Current (Bx-Sy)", type => "signed_word_1", modifier => "*0.000122", unit => "A"}
            ] },
            
            "Freeze frame Catalyst Temperature Bank 1, Sensor 1" => { command => "02 3C", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Freeze frame Catalyst Temperature Bank 1, Sensor 2" => { command => "02 3D", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Freeze frame Catalyst Temperature Bank 2, Sensor 1" => { command => "02 3E", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },
            "Freeze frame Catalyst Temperature Bank 2, Sensor 2" => { command => "02 3F", result => [{type => "word_0", modifier => "/10-40", unit => "°C"}] },

            "02 PIDs supported (41-60)" => { command => "02 40",
            result => [
            {name => "Freeze frame Monitor status this driving cycle", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Freeze frame Control Module Voltage", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Freeze frame Absolute Load Value", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Freeze frame Commanded Equivalence Ratio", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Freeze frame Relative Throttle Position", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Freeze frame Ambiant Air Temperature", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Freeze frame Absolute Throttle Position B", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Freeze frame Absolute Throttle Position C", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Freeze frame Absolute Throttle Position D", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Freeze frame Absolute Throttle Position E", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Freeze frame Absolute Throttle Position F", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Freeze frame Commanded Throttle Actuator Control", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Freeze frame Minutes run by the engine while MIL activated", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Freeze frame Time since diagnostic trouble codes cleared", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Freeze frame External Test Equipment Configuration Information #1", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Freeze frame External Test Equipment Configuration Information #2", type => "bool_1", modifier => "&1", unit => ""},
            
            {name => "Freeze frame Type of fuel currently being utilized by the vehicle", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Freeze frame Alcohol Fuel Percentage", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Freeze frame Absolute Evap System Vapour Pressure", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Freeze frame Evap System Vapour Pressure", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Freeze frame Short Term Secondary O2 Sensor Fuel Trim - Bank 1", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Freeze frame Long Term Secondary O2 Sensor Fuel Trim - Bank 1", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Freeze frame Short Term Secondary O2 Sensor Fuel Trim - Bank 2", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Freeze frame Long Term Secondary O2 Sensor Fuel Trim - Bank 2", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Freeze frame Short Term Secondary O2 Sensor Fuel Trim - Bank 3", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Freeze frame Long Term Secondary O2 Sensor Fuel Trim - Bank 3", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Freeze frame Short Term Secondary O2 Sensor Fuel Trim - Bank 4", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Freeze frame Long Term Secondary O2 Sensor Fuel Trim - Bank 4", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "Freeze frame Fuel Rail Pressure (absolute)", type => "bool_3", modifier => "&128", unit => ""},
            {name => "Freeze frame Relative Accelerator Pedal Position", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "Freeze frame ", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "Freeze frame ", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "Freeze frame ", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "Freeze frame ", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "Freeze frame ", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "02 PIDs supported (61-80)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            
            "Freeze frame Monitor status this driving cycle" => { command => "02 41",
            result => [
            {name => "Freeze frame Misfire monitoring Enable Status", type => "bool_1", modifier => "&1", unit => ""},
            {name => "Freeze frame Fuel system monitoring Enable Status", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Freeze frame Comprehensive component monitoring Enable Status", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Freeze frame Misfire monitoring Completion Status", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Freeze frame Fuel system monitoring Completion Status", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Freeze frame Comprehensive component monitoring Completion Status", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Freeze frame Catalyst monitoring Enable Status", type => "bool_2", modifier => "&1", unit => ""},
            {name => "Freeze frame Heated catalyst monitoring Enable Status", type => "bool_2", modifier => "&2", unit => ""},
            {name => "Freeze frame Evaporative system monitoring Enable Status", type => "bool_2", modifier => "&4", unit => ""},
            {name => "Freeze frame Secondary air system monitoring Enable Status", type => "bool_2", modifier => "&8", unit => ""},
            {name => "Freeze frame A/C system refrigerant monitoring Enable Status", type => "bool_2", modifier => "&16", unit => ""},
            {name => "Freeze frame Oxygen sensor monitoring Enable Status", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Freeze frame Oxygen sensor heater monitoring Enable Status", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Freeze frame EGR monitoring Enable Status", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Freeze frame Catalyst monitoring Completion Status", type => "bool_3", modifier => "&1", unit => ""},
            {name => "Freeze frame Heated catalyst monitoring Completion Status", type => "bool_3", modifier => "&2", unit => ""},
            {name => "Freeze frame Evaporative system monitoring Completion Status", type => "bool_3", modifier => "&4", unit => ""},
            {name => "Freeze frame Secondary air system monitoring Completion Status", type => "bool_3", modifier => "&8", unit => ""},
            {name => "Freeze frame A/C system refrigerant monitoring Completion Status", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Freeze frame Oxygen sensor monitoring Completion Status", type => "bool_3", modifier => "&32", unit => ""},
            {name => "Freeze frame Oxygen sensor heater monitoring Completion Status", type => "bool_3", modifier => "&64", unit => ""},
            {name => "Freeze frame EGR monitoring Completion Status", type => "bool_3", modifier => "&128", unit => ""}
            ] },
            
            "Freeze frame Control Module Voltage" => { command => "02 42", result => [{type => "word_0", modifier => "*0.001", unit => "V"}] },
            "Freeze frame Absolute Load Value" => { command => "02 43", result => [{type => "word_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Commanded Equivalence Ratio" => { command => "02 44", result => [{type => "word_0", modifier => "*0.0000305", unit => ""}] },
            "Freeze frame Relative Throttle Position" => { command => "02 45", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Ambiant Air Temperature" => { command => "02 46", result => [{type => "byte_0", modifier => "-40", unit => "°C"}] },
            "Freeze frame Absolute Throttle Position B" => { command => "02 47", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Absolute Throttle Position C" => { command => "02 48", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Accelerator Pedal Position D" => { command => "02 49", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Accelerator Pedal Position E" => { command => "02 4A", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Accelerator Pedal Position F" => { command => "02 4B", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Commanded Throttle Actuator Control" => { command => "02 4C", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Minutes run by the engine while MIL activated" => { command => "02 4D", result => [{type => "word_0", modifier => "+0", unit => "min"}] },
            "Freeze frame Time since diagnostic trouble codes cleared" => { command => "02 4E", result => [{type => "word_0", modifier => "+0", unit => "min"}] },
            "Freeze frame External Test Equipment Configuration Information #1" => { command => "02 4F",
            result => [
            {name => "Freeze frame Maximum value for Equivalence Ratio", type => "byte_0", modifier => "+0", unit => ""},
            {name => "Freeze frame Maximum value for Oxygen Sensor Voltage", type => "byte_1", modifier => "+0", unit => "V"},
            {name => "Freeze frame Maximum value for Oxygen Sensor Current", type => "byte_2", modifier => "+0", unit => "mA"},
            {name => "Freeze frame Maximum value for Intake Manifold Absolute Pressure", type => "byte_3", modifier => "+0", unit => "kPa"},
            ] },
            "Freeze frame External Test Equipment Configuration Information #2" => { command => "02 50",
            result => [
            {name => "Freeze frame Maximum value for Air Flow Rate from Mass Air Flow Sensor", type => "byte_0", modifier => "+0", unit => "g/s"}
            ] },

            "Freeze frame Type of fuel currently being utilized by the vehicle" => { command => "02 51", result => [{type => "byte_0", modifier => "+0", unit => "",
            alternatives => [
            {value => 1, meaning => "Gasoline/petrol"},
            {value => 2, meaning => "Methanol"},
            {value => 3, meaning => "Ethanol"},
            {value => 4, meaning => "Diesel"},
            {value => 5, meaning => "Liquefied Petroleum Gas (LPG)"},
            {value => 6, meaning => "Compressed Natural Gas (CNG)"},
            {value => 7, meaning => "Propane"},
            {value => 8, meaning => "Battery/electric"},
            {value => 9, meaning => "Bi-fuel vehicle using gasoline"},
            {value => 10, meaning => "Bi-fuel vehicle using methanol"},
            {value => 11, meaning => "Bi-fuel vehicle using ethanol"},
            {value => 12, meaning => "Bi-fuel vehicle using LPG"},
            {value => 13, meaning => "Bi-fuel vehicle using CNG"},
            {value => 14, meaning => "Bi-fuel vehicle using propane"},
            {value => 15, meaning => "Bi-fuel vehicle using battery"}
            ] }
            ] },
            
            "Freeze frame Alcohol Fuel Percentage" => { command => "02 52", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },
            "Freeze frame Absolute Evap System Vapour Pressure" => { command => "02 53", result => [{type => "word_0", modifier => "/200", unit => "kPa"}] },
            "Freeze frame Evap System Vapour Pressure" => { command => "02 54", result => [{type => "signed_word_0", modifier => "+1", unit => "Pa"}] },
            "Freeze frame Short Term Secondary O2 Sensor Fuel Trim - Bank 1" => { command => "02 55", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Freeze frame Long Term Secondary O2 Sensor Fuel Trim - Bank 1" => { command => "02 56", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Freeze frame Short Term Secondary O2 Sensor Fuel Trim - Bank 2" => { command => "02 57", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },
            "Freeze frame Long Term Secondary O2 Sensor Fuel Trim - Bank 2" => { command => "02 58", result => [{type => "signed_byte_0", modifier => "*100/128", unit => "%"}] },

            "Freeze frame Short Term Secondary O2 Sensor Fuel Trim - Bank 3" => { command => "02 55", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Freeze frame Long Term Secondary O2 Sensor Fuel Trim - Bank 3" => { command => "02 56", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Freeze frame Short Term Secondary O2 Sensor Fuel Trim - Bank 4" => { command => "02 57", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },
            "Freeze frame Long Term Secondary O2 Sensor Fuel Trim - Bank 4" => { command => "02 58", result => [{type => "signed_byte_1", modifier => "*100/128", unit => "%"}] },

            "Freeze frame Fuel Rail Pressure (absolute)" => { command => "02 59", result => [{type => "word_0", modifier => "*10", unit => "kPa"}] },
            "Freeze frame Relative Accelerator Pedal Position" => { command => "02 5A", result => [{type => "byte_0", modifier => "*100/255", unit => "%"}] },

            

            "05 TIDs supported (01-20)" => { command => "05 00",
            result => [
            {name => "Rich to lean sensor threshold voltage", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Lean to rich sensor threshold voltage", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Low sensor voltage for switch time calculation", type => "bool_0", modifier => "&32", unit => ""},
            {name => "High sensor voltage for switch time calculation", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Rich to lean sensor switch time", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Lean to rich sensor switch time", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Minimum sensor voltage for test cycle", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Maximum sensor voltage for test cycle", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Time between sensor transitions", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Sensor period", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "05 TIDs supported (21-40)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Rich to lean sensor threshold voltage" => { command => "05 01", result => [{type => "word_0", modifier => "*0.005", unit => "V"}] },
            "Lean to rich sensor threshold voltage" => { command => "05 02", result => [{type => "word_0", modifier => "*0.005", unit => "V"}] },
            "Low sensor voltage for switch time calculation" => { command => "05 03", result => [{type => "word_0", modifier => "*0.005", unit => "V"}] },
            "High sensor voltage for switch time calculation" => { command => "05 04", result => [{type => "word_0", modifier => "*0.005", unit => "V"}] },
            "Rich to lean sensor switch time" => { command => "05 05", result => [{type => "word_0", modifier => "*0.004", unit => "s"}] },
            "Lean to rich sensor switch time" => { command => "05 06", result => [{type => "word_0", modifier => "*0.004", unit => "s"}] },
            "Minimum sensor voltage for test cycle" => { command => "05 07", result => [{type => "word_0", modifier => "*0.005", unit => "V"}] },
            "Maximum sensor voltage for test cycle" => { command => "05 08", result => [{type => "word_0", modifier => "*0.005", unit => "V"}] },
            "Time between sensor transitions" => { command => "05 09", result => [{type => "word_0", modifier => "*0.04", unit => "s"}] },
            "Sensor period" => { command => "05 0A", result => [{type => "word_0", modifier => "*0.04", unit => "s"}] },


            "06 MIDs supported (01-20)" => { command => "06 00",
            result => [
            {name => "Oxygen Sensor Monitor Bank 1 - Sensor 1", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 1 - Sensor 2", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 1 - Sensor 3", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 1 - Sensor 4", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 2 - Sensor 1", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 2 - Sensor 2", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 2 - Sensor 3", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 2 - Sensor 4", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Oxygen Sensor Monitor Bank 3 - Sensor 1", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 3 - Sensor 2", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 3 - Sensor 3", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 3 - Sensor 4", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 4 - Sensor 1", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 4 - Sensor 2", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 4 - Sensor 3", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Oxygen Sensor Monitor Bank 4 - Sensor 4", type => "bool_1", modifier => "&1", unit => ""},

# 					11 -1F reserved by ISO/SAE
#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
            {name => "06 MIDs supported (21-40)", type => "bool_3", modifier => "&1", unit => ""},
            ] },
            
            "Oxygen Sensor Monitor Bank 1 - Sensor 1" => { command => "06 01", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 1 - Sensor 2" => { command => "06 02", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 1 - Sensor 3" => { command => "06 03", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 1 - Sensor 4" => { command => "06 04", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 2 - Sensor 1" => { command => "06 05", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 2 - Sensor 2" => { command => "06 06", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 2 - Sensor 3" => { command => "06 07", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 2 - Sensor 4" => { command => "06 08", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 3 - Sensor 1" => { command => "06 09", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 3 - Sensor 2" => { command => "06 0A", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 3 - Sensor 3" => { command => "06 0B", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 3 - Sensor 4" => { command => "06 0C", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 4 - Sensor 1" => { command => "06 0D", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 4 - Sensor 2" => { command => "06 0E", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 4 - Sensor 3" => { command => "06 0F", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Monitor Bank 4 - Sensor 4" => { command => "06 10", result => [{type => "word_0", modifier => "*1", unit => ""}] },

            "06 MIDs supported (21-40)" => { command => "06 20",
            result => [
            {name => "Catalyst Monitor Bank 1", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Catalyst Monitor Bank 2", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Catalyst Monitor Bank 3", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Catalyst Monitor Bank 4", type => "bool_0", modifier => "&16", unit => ""},
#            {name => "", type => "bool_0", modifier => "&8", unit => ""},
#            {name => "", type => "bool_0", modifier => "&4", unit => ""},
#            {name => "", type => "bool_0", modifier => "&2", unit => ""},
#            {name => "", type => "bool_0", modifier => "&1", unit => ""},

#            {name => "", type => "bool_1", modifier => "&128", unit => ""},
#            {name => "", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},

            {name => "EGR Monitor Bank 1", type => "bool_2", modifier => "&128", unit => ""},
            {name => "EGR Monitor Bank 2", type => "bool_2", modifier => "&64", unit => ""},
            {name => "EGR Monitor Bank 3", type => "bool_2", modifier => "&32", unit => ""},
            {name => "EGR Monitor Bank 4", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
            {name => "EVAP Monitor (Cap Off)", type => "bool_3", modifier => "&128", unit => ""},
            {name => "EVAP Monitor (0,090)", type => "bool_3", modifier => "&64", unit => ""},
            {name => "EVAP Monitor (0,040)", type => "bool_3", modifier => "&32", unit => ""},
            {name => "EVAP Monitor (0,020)", type => "bool_3", modifier => "&16", unit => ""},
            {name => "Purge Flow Monitor", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
            {name => "06 MIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            "Catalyst Monitor Bank 1" => { command => "06 21", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Catalyst Monitor Bank 2" => { command => "06 22", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Catalyst Monitor Bank 3" => { command => "06 23", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Catalyst Monitor Bank 4" => { command => "06 24", result => [{type => "word_0", modifier => "*1", unit => ""}] },

            "EGR Monitor Bank 1" => { command => "06 31", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "EGR Monitor Bank 2" => { command => "06 32", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "EGR Monitor Bank 3" => { command => "06 33", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "EGR Monitor Bank 4" => { command => "06 34", result => [{type => "word_0", modifier => "*1", unit => ""}] },

            "EVAP Monitor (Cap Off)" => { command => "06 39", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "EVAP Monitor (0,090)" => { command => "06 3A", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "EVAP Monitor (0,040)" => { command => "06 3B", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "EVAP Monitor (0,020)" => { command => "06 3C", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Purge Flow Monitor" => { command => "06 3D", result => [{type => "word_0", modifier => "*1", unit => ""}] },

            
            "06 MIDs supported (41-60)" => { command => "06 40",
            result => [
            {name => "Oxygen Sensor Heater Monitor Bank 1 - Sensor 1", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 1 - Sensor 2", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 1 - Sensor 3", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 1 - Sensor 4", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 2 - Sensor 1", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 2 - Sensor 2", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 2 - Sensor 3", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 2 - Sensor 4", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Oxygen Sensor Heater Monitor Bank 3 - Sensor 1", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 3 - Sensor 2", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 3 - Sensor 3", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 3 - Sensor 4", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 4 - Sensor 1", type => "bool_1", modifier => "&8", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 4 - Sensor 2", type => "bool_1", modifier => "&4", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 4 - Sensor 3", type => "bool_1", modifier => "&2", unit => ""},
            {name => "Oxygen Sensor Heater Monitor Bank 4 - Sensor 4", type => "bool_1", modifier => "&1", unit => ""},

#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
            {name => "06 MIDs supported (61-80)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            "Oxygen Sensor Heater Monitor Bank 1 - Sensor 1" => { command => "06 41", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 1 - Sensor 2" => { command => "06 42", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 1 - Sensor 3" => { command => "06 43", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 1 - Sensor 4" => { command => "06 44", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 2 - Sensor 1" => { command => "06 45", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 2 - Sensor 2" => { command => "06 46", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 2 - Sensor 3" => { command => "06 47", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 2 - Sensor 4" => { command => "06 48", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 3 - Sensor 1" => { command => "06 49", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 3 - Sensor 2" => { command => "06 4A", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 3 - Sensor 3" => { command => "06 4B", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 3 - Sensor 4" => { command => "06 4C", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 4 - Sensor 1" => { command => "06 4D", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 4 - Sensor 2" => { command => "06 4E", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 4 - Sensor 3" => { command => "06 4F", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Oxygen Sensor Heater Monitor Bank 4 - Sensor 4" => { command => "06 50", result => [{type => "word_0", modifier => "*1", unit => ""}] },



            "06 MIDs supported (61-80)" => { command => "06 60",
            result => [
            {name => "Heated Catalyst Monitor Bank 1", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Heated Catalyst Monitor Bank 2", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Heated Catalyst Monitor Bank 3", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Heated Catalyst Monitor Bank 4", type => "bool_0", modifier => "&16", unit => ""},
#            {name => "", type => "bool_0", modifier => "&8", unit => ""},
#            {name => "", type => "bool_0", modifier => "&4", unit => ""},
#            {name => "", type => "bool_0", modifier => "&2", unit => ""},
#            {name => "", type => "bool_0", modifier => "&1", unit => ""},

#            {name => "", type => "bool_1", modifier => "&128", unit => ""},
#            {name => "", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},

            {name => "Secondary Air Monitor 1", type => "bool_2", modifier => "&128", unit => ""},
            {name => "Secondary Air Monitor 2", type => "bool_2", modifier => "&64", unit => ""},
            {name => "Secondary Air Monitor 3", type => "bool_2", modifier => "&32", unit => ""},
            {name => "Secondary Air Monitor 4", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
            {name => "06 MIDs supported (81-A0)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            "Heated Catalyst Monitor Bank 1" => { command => "06 61", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Heated Catalyst Monitor Bank 2" => { command => "06 62", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Heated Catalyst Monitor Bank 3" => { command => "06 63", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Heated Catalyst Monitor Bank 4" => { command => "06 64", result => [{type => "word_0", modifier => "*1", unit => ""}] },

            "Secondary Air Monitor 1" => { command => "06 71", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Secondary Air Monitor 2" => { command => "06 72", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Secondary Air Monitor 3" => { command => "06 73", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Secondary Air Monitor 4" => { command => "06 74", result => [{type => "word_0", modifier => "*1", unit => ""}] },

            "06 MIDs supported (81-A0)" => { command => "06 80",
            result => [
            {name => "Fuel System Monitor Bank 1", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Fuel System Monitor Bank 2", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Fuel System Monitor Bank 3", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Fuel System Monitor Bank 4", type => "bool_0", modifier => "&16", unit => ""},
#            {name => "", type => "bool_0", modifier => "&8", unit => ""},
#            {name => "", type => "bool_0", modifier => "&4", unit => ""},
#            {name => "", type => "bool_0", modifier => "&2", unit => ""},
#            {name => "", type => "bool_0", modifier => "&1", unit => ""},

#            {name => "", type => "bool_1", modifier => "&128", unit => ""},
#            {name => "", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},

#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
            {name => "06 MIDs supported (A1-C0)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            "Fuel System Monitor Bank 1" => { command => "06 81", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Fuel System Monitor Bank 2" => { command => "06 82", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Fuel System Monitor Bank 3" => { command => "06 83", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Fuel System Monitor Bank 4" => { command => "06 84", result => [{type => "word_0", modifier => "*1", unit => ""}] },


            "06 MIDs supported (A1-C0)" => { command => "06 A0",
            result => [
            {name => "Misfire Monitor General Data", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Misfire Cylinder 1 Data", type => "bool_0", modifier => "&64", unit => ""},
            {name => "Misfire Cylinder 2 Data", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Misfire Cylinder 3 Data", type => "bool_0", modifier => "&16", unit => ""},
            {name => "Misfire Cylinder 4 Data", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Misfire Cylinder 5 Data", type => "bool_0", modifier => "&4", unit => ""},
            {name => "Misfire Cylinder 6 Data", type => "bool_0", modifier => "&2", unit => ""},
            {name => "Misfire Cylinder 7 Data", type => "bool_0", modifier => "&1", unit => ""},

            {name => "Misfire Cylinder 8 Data", type => "bool_1", modifier => "&128", unit => ""},
            {name => "Misfire Cylinder 9 Data", type => "bool_1", modifier => "&64", unit => ""},
            {name => "Misfire Cylinder 10 Data", type => "bool_1", modifier => "&32", unit => ""},
            {name => "Misfire Cylinder 11 Data", type => "bool_1", modifier => "&16", unit => ""},
            {name => "Misfire Cylinder 12 Data", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},

#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
            {name => "06 MIDs supported (C1-E0)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            "Misfire Monitor General Data" => { command => "06 A1", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 1 Data" => { command => "06 A2", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 2 Data" => { command => "06 A3", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 3 Data" => { command => "06 A4", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 4 Data" => { command => "06 A5", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 5 Data" => { command => "06 A6", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 6 Data" => { command => "06 A7", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 7 Data" => { command => "06 A8", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 8 Data" => { command => "06 A9", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 9 Data" => { command => "06 AA", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 10 Data" => { command => "06 AB", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 11 Data" => { command => "06 AC", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Misfire Cylinder 12 Data" => { command => "06 AD", result => [{type => "word_0", modifier => "*1", unit => ""}] },



            "06 MIDs supported (C1-E0)" => { command => "06 C0",
            result => [
#            {name => "", type => "bool_0", modifier => "&128", unit => ""},
#            {name => "", type => "bool_0", modifier => "&64", unit => ""},
#            {name => "", type => "bool_0", modifier => "&32", unit => ""},
#            {name => "", type => "bool_0", modifier => "&16", unit => ""},
#            {name => "", type => "bool_0", modifier => "&8", unit => ""},
#            {name => "", type => "bool_0", modifier => "&4", unit => ""},
#            {name => "", type => "bool_0", modifier => "&2", unit => ""},
#            {name => "", type => "bool_0", modifier => "&1", unit => ""},

#            {name => "", type => "bool_1", modifier => "&128", unit => ""},
#            {name => "", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},

#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
            {name => "06 MIDs supported (E1-FF)", type => "bool_3", modifier => "&1", unit => ""},
            ] },

            "06 MIDs supported (E1-FF)" => { command => "06 E0",
            result => [
#            {name => "Vehicle manufacturer defined OBDMID E1", type => "bool_0", modifier => "&128", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID E2", type => "bool_0", modifier => "&64", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID E3", type => "bool_0", modifier => "&32", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID E4", type => "bool_0", modifier => "&16", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID E5", type => "bool_0", modifier => "&8", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID E6", type => "bool_0", modifier => "&4", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID E7", type => "bool_0", modifier => "&2", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID E8", type => "bool_0", modifier => "&1", unit => ""},

#            {name => "Vehicle manufacturer defined OBDMID E9", type => "bool_1", modifier => "&128", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID EA", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID EB", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID EC", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID ED", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID EE", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID EF", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID F0", type => "bool_1", modifier => "&1", unit => ""},

#            {name => "Vehicle manufacturer defined OBDMID F1", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID F2", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID F3", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID F4", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID F5", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID F6", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID F7", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID F8", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "Vehicle manufacturer defined OBDMID F9", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID FA", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID FB", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID FC", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID FD", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID FE", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "Vehicle manufacturer defined OBDMID FF", type => "bool_3", modifier => "&2", unit => ""},
            ] },

            "Vehicle manufacturer defined OBDMID E1" => { command => "06 E1", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID E2" => { command => "06 E2", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID E3" => { command => "06 E3", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID E4" => { command => "06 E4", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID E5" => { command => "06 E5", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID E6" => { command => "06 E6", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID E7" => { command => "06 E7", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID E8" => { command => "06 E8", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID E9" => { command => "06 E9", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID EA" => { command => "06 EA", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID EB" => { command => "06 EB", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID EC" => { command => "06 EC", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID ED" => { command => "06 ED", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID EE" => { command => "06 EE", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID EF" => { command => "06 EF", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F0" => { command => "06 F0", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F1" => { command => "06 F1", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F2" => { command => "06 F2", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F3" => { command => "06 F3", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F4" => { command => "06 F4", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F5" => { command => "06 F5", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F6" => { command => "06 F6", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F7" => { command => "06 F7", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F8" => { command => "06 F8", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID F9" => { command => "06 F9", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID FA" => { command => "06 FA", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID FB" => { command => "06 FB", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID FC" => { command => "06 FC", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID FD" => { command => "06 FD", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID FE" => { command => "06 FE", result => [{type => "word_0", modifier => "*1", unit => ""}] },
            "Vehicle manufacturer defined OBDMID FF" => { command => "06 FF", result => [{type => "word_0", modifier => "*1", unit => ""}] },
                                                                         


            "09 PIDs supported (01-20)" => { command => "09 00", available => 1,
            result => [
            {name => "MessageCount VIN", type => "bool_0", modifier => "&128", unit => ""},
            {name => "Vehicle Identification Number", type => "bool_0", modifier => "&64", unit => ""},
            {name => "MessageCount CALID", type => "bool_0", modifier => "&32", unit => ""},
            {name => "Calibration Identifications", type => "bool_0", modifier => "&16", unit => ""},
            {name => "MessageCount CVN", type => "bool_0", modifier => "&8", unit => ""},
            {name => "Calibration Verification Numbers", type => "bool_0", modifier => "&4", unit => ""},
            {name => "MessageCount IPT", type => "bool_0", modifier => "&2", unit => ""},
            {name => "In-use Performance Tracking", type => "bool_0", modifier => "&1", unit => ""},

            {name => "MessageCount ECUNAME", type => "bool_1", modifier => "&128", unit => ""},
            {name => "ECUNAME", type => "bool_1", modifier => "&64", unit => ""},
#            {name => "", type => "bool_1", modifier => "&32", unit => ""},
#            {name => "", type => "bool_1", modifier => "&16", unit => ""},
#            {name => "", type => "bool_1", modifier => "&8", unit => ""},
#            {name => "", type => "bool_1", modifier => "&4", unit => ""},
#            {name => "", type => "bool_1", modifier => "&2", unit => ""},
#            {name => "", type => "bool_1", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_2", modifier => "&128", unit => ""},
#            {name => "", type => "bool_2", modifier => "&64", unit => ""},
#            {name => "", type => "bool_2", modifier => "&32", unit => ""},
#            {name => "", type => "bool_2", modifier => "&16", unit => ""},
#            {name => "", type => "bool_2", modifier => "&8", unit => ""},
#            {name => "", type => "bool_2", modifier => "&4", unit => ""},
#            {name => "", type => "bool_2", modifier => "&2", unit => ""},
#            {name => "", type => "bool_2", modifier => "&1", unit => ""},
            
#            {name => "", type => "bool_3", modifier => "&128", unit => ""},
#            {name => "", type => "bool_3", modifier => "&64", unit => ""},
#            {name => "", type => "bool_3", modifier => "&32", unit => ""},
#            {name => "", type => "bool_3", modifier => "&16", unit => ""},
#            {name => "", type => "bool_3", modifier => "&8", unit => ""},
#            {name => "", type => "bool_3", modifier => "&4", unit => ""},
#            {name => "", type => "bool_3", modifier => "&2", unit => ""},
#            {name => "09 PIDs supported (41-60)", type => "bool_3", modifier => "&1", unit => ""},
            ] },            

            "MessageCount VIN" => { command => "09 01", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Vehicle Identification Number" => { command => "09 02", result => [{type => "string_1", modifier => "", unit => ""}] },
            "MessageCount CALID" => { command => "09 03", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Calibration Identifications" => { command => "09 04", result => [{type => "string_1", modifier => "", unit => ""}] },
            "MessageCount CVN" => { command => "09 05", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "Calibration Verification Numbers" => { command => "09 06", result => [{type => "dword_0", modifier => "+0", unit => ""}] },
            "MessageCount IPT" => { command => "09 07", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "In-use Performance Tracking" => { command => "09 08",
            result => [
            {name => "OBD Monitoring Conditions Encountered Counts", type => "word_0", modifier => "+0", unit => ""},
            {name => "Ignition Counter", type => "word_1", modifier => "+0", unit => ""},
            {name => "Catalyst Monitor Completion Counts Bank 1", type => "word_2", modifier => "+0", unit => ""},
            {name => "Catalyst Monitor Conditions Encountered Counts Bank 1", type => "word_3", modifier => "+0", unit => ""},
            {name => "Catalyst Monitor Completion Counts Bank 2", type => "word_4", modifier => "+0", unit => ""},
            {name => "Catalyst Monitor Conditions Encountered Counts Bank 2", type => "word_5", modifier => "+0", unit => ""},
            {name => "O2 Sensor Monitor Completion Counts Bank 1", type => "word_6", modifier => "+0", unit => ""},
            {name => "O2 Sensor Monitor Conditions Encountered Counts Bank 1", type => "word_7", modifier => "+0", unit => ""},
            {name => "O2 Sensor Monitor Completion Counts Bank 2", type => "word_8", modifier => "+0", unit => ""},
            {name => "O2 Sensor Monitor Conditions Encountered Counts Bank 2", type => "word_9", modifier => "+0", unit => ""},
            {name => "EGR Monitor Completion Condition Counts", type => "word_10", modifier => "+0", unit => ""},
            {name => "EGR Monitor Conditions Encountered Counts", type => "word_11", modifier => "+0", unit => ""},
            {name => "AIR Monitor Completion Condition Counts (Secondary Air)", type => "word_12", modifier => "+0", unit => ""},
            {name => "AIR Monitor Conditions Encountered Counts (Secondary Air)", type => "word_13", modifier => "+0", unit => ""},
            {name => "EVAP Monitor Completion Condition Counts", type => "word_14", modifier => "+0", unit => ""},
            {name => "EVAP Monitor Conditions Encountered Counts", type => "word_15", modifier => "+0", unit => ""}
            ] },
            "MessageCount ECUNAME" => { command => "09 09", result => [{type => "byte_0", modifier => "+0", unit => ""}] },
            "ECUNAME" => { command => "09 0A", result => [{type => "string_0", modifier => "", unit => ""}] },
            
            };


	if (defined($replay_filename))
	{
    $self->Replay($replay_filename);

#    $self->ShowReadableValues();
#print Dumper($self->{'get'});
	}
  else
  {
    $self->OpenPort($port_name);
    $self->ConfigureDevice();
    $self->FindAvailableCommands();
  }

	return $self;
}


#*****************************************************************

# Try to find an ELM module on a COM port.
# If a $port_name is supplied, start with that one and work upwards.

sub OpenPort
{
  my ($self, $port_name) = @_;
  my $quiet = 0;
  my $port = -1;
  my $port_count = 0;

  if (!defined($port_name) || $port_name eq "")
  {
    if ($^O eq "MSWin32")
    {
      $port_name = "COM1";
    }
    else
    {
      $port_name = "/dev/ttyUSB0";
    }
  }

 	my $port_number = $port_name;
	$port_number =~ s/[^0-9]//g;  # Strip everything that isn't numeric

	my $port_text = $port_name;
	$port_text =~ s/[0-9]//g;     # Strip everything that is numeric

  do
  {
    $port_name = $port_text.$port_number;
    
    if ($^O eq "MSWin32")
    {
      $port = Win32::SerialPort->new ($port_name);
    }
    else
    {
      $port = Device::SerialPort->new($port_name, $quiet);
    }

    if (defined($port))
    {
      $port->user_msg(1); 	    # misc. warnings
      $port->error_msg(1); 	    # hardware and data errors

      $port->baudrate(38400);
      $port->parity("none");
      $port->parity_enable(0);  # for any parity except "none"
      $port->databits(8);
      $port->stopbits(1);
      $port->handshake('none');

      $port->write_settings;

      $self->{'port'} = $port;
      $self->Command("AT Z");   # Reset Device
      foreach (@{$self->{'response'}})
      {
        if (substr($_, 0, 5) eq "ELM32")  # Allow 328 & 329 as well
        {
          $self->{'ELM_type'} = substr($_, 0, 6);
        }
      }
      if ($self->{'ELM_type'} eq "NONE" && $self->{'debug_level'} > 0)
      {
        print "Can't find an ELM module on $port_name\n";
      }
    }
    else
    {
      if ($self->{'debug_level'} > 0)
      {
        print "Can't open $port_name: $!\n";
      }
      $self->{'port'} = -1;
    }

    $port_number++;
    $port_count++;
  } until(($self->{'port'} != -1 && $self->{'ELM_type'} ne "NONE") || $port_count > $max_ports_to_search);

  if ($self->{'ELM_type'} eq "NONE")
  {
    $self->{'port'} = -1;
    die "Couldn't find an ELM module!\n"; 
  }
}


#*****************************************************************

=head2 PortOK

Returns 1 if the serial port and ELM module are working or 0 if no ELM device could be connected to.

 $obd->PortOK();
=cut

sub PortOK
{
	my ($self) = @_;

  if ($self->{'port'} == -1)
  {
    if ($self->{'debug_level'} > 0) { print "Serial port not initialised!\n"; }
    return 0;
  }
  else
  {
    return 1;
  }
}


#*****************************************************************

# Set up the ELM module to return data in the required form

sub ConfigureDevice
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "ConfigureDevice\n"; }
  
  if ($self->PortOK)
  {
    $self->Command("AT E0");   # Turn echo off

    $self->Command("AT L0");   # Turn linefeeds off

    $self->Command("AT SP 0");  # Set Protocol to auto

    $self->Command("AT DPN");  # Display Protocol number

    $self->Command("AT DP");  # Display Protocol

    $self->Command("AT H1");  # Turn headers on

    $self->Command("01 00");  # Prepare to talk and get available PID's
  }
}


#*****************************************************************

# Query the ECU to find out which commands are supported and
# annotate the value entries in the 'get' structure with the 
# 'available' flag set to 0 (not supported) or 1 (supported).

sub FindAvailableCommands
{
	my ($self) = @_;

	my @commands = ( "01 PIDs supported (01-20)",
	                 "02 PIDs supported (01-20)",
	                 "09 PIDs supported (01-20)"
	               );

  if ($self->{'debug_level'} > 0) { print "FindAvailableCommands:\n"; }

	if ($self->{'bus_type'} eq "CAN")
	{
		push @commands, "06 MIDs supported (01-20)";	# Command only supported by CAN systems
	}
	else
	{
		push @commands, "05 TIDs supported (01-20)";	# Command only supported by non-CAN systems
#		push @commands, "06 TIDs supported (01-20)";	# Command only supported by non-CAN systems
	}

	foreach my $next_command (@commands)
	{
		do
		{
			$next_command = $self->ProcessAvailableCommands($next_command);
		} while (defined($next_command));
	}
}


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Query the ECU to find out which commands are supported and
# annotate the value entries in the 'get' structure with the 
# 'available' flag set to 0 (not supported) or 1 (supported).

sub ProcessAvailableCommands
{
	my ($self, $command) = @_;

  my $next_command = undef;
  
  if ($self->{'debug_level'} > 0) { print "~ProcessAvailableCommands: $command\n"; }

	$self->{'get'}->{$command}->{'available'} = 1;	# Flag the command as available
	
  my $response = $self->Read($command);

  if ($self->{'debug_level'} > 1) { print "$command\n"; }

  if ($response->{'status'} == 0)
  {
    foreach my $result (@{$response->{'results'}})
    {
      $self->{'get'}->{$result->{'name'}}->{'available'} = $result->{'value'};
      if ($self->{'debug_level'} > 1) { print "$result->{'address'} - $result->{'name'}: $result->{'value'} $result->{'unit'}\n"; }

      if (substr($result->{'name'}, 4, 15) eq "IDs supported (" && $result->{'value'} == 1)
      {
        $next_command = $result->{'name'};
      }
      elsif ( $result->{'value'} == 1 &&
						((substr($result->{'name'}, -2, 2) eq "13" && $self->{'get'}->{'Location of oxygen sensors 13'}->{'available'} == 1)
					|| (substr($result->{'name'}, -2, 2) eq "1D" && $self->{'get'}->{'Location of oxygen sensors 1D'}->{'available'} == 1))
					  )
      {
        my $new_name = substr($result->{'name'}, 0, length($result->{'name'})-3);
        
				if ($self->{'debug_level'} > 2)
				{
					print "Old name: >$result->{'name'}<\n";
					print "New name: >$new_name<\n";
				}
				
        $self->{'get'}->{$new_name} = delete($self->{'get'}->{$result->{'name'}});
				if ($self->{'debug_level'} > 3)
				{
					print Dumper($self->{'get'}->{$new_name});
				}
      }
    }
  }
  else
  {
		# Flag command as unavailable
		$self->{'get'}->{$command}->{'available'} = 0;
	}

  return $next_command;
}


#*****************************************************************

=head2 ShowReadableValues

Displays the list of values that can be read from the ELM/ECU.

 $obd->ShowReadableValues();
=cut

sub ShowReadableValues
{
	my ($self) = @_;

  print "Values that the Read and Show functions can fetch from this vehicle:\n\n";

  while ( my ($parameter_name, $definition) = each %{$self->{'get'}} )
  {
  	if (exists($definition->{'available'}) && $definition->{'available'} == 1)
  	{
		  print "Value: \"$parameter_name\"\n";
		  print "Returns:\n";
		  foreach my $result (@{$definition->{'result'}})
		  {
		    if (exists($result->{'name'}))
		    {
		      print "$result->{'name'}";
		    }
		    else
		    {
		      print "$parameter_name";
		    }
		    if (exists($result->{'unit'}) && $result->{'unit'} ne "")
		    {
		      print " ($result->{'unit'})";
		    }
		    elsif (substr($result->{'type'}, 0, 4) eq "bool")
		    {
		      print " (0 or 1)";
		    }
		    print "\n";
		  }
		  print "\n";
		}
  }
}


#*****************************************************************

# Process a file containing debugging output and replay the commands
# through the module.

sub Replay
{
  my ($self, $replay_filename) = @_;
  $self->{'replay_file'} = 1;
  
  my $status = 0;
  my $replay_command="";

  # Open the file containing the command and response data
  open (REPLAYFILE, $replay_filename);

  my $get_command = 0;
  my $seek_response = 1;
  my $read_response = 2;
  my $get_result = 4;
  
  my $replay_state = $get_command; 

  while (<REPLAYFILE>)
  { 
    # Iterate through issuing commands and parsing responses
    $_ =~ s/\r?\n//g;   # Strip all carriage returns

    if ($replay_state == $get_command)
    {
      if (substr($_, 0, 1) eq "~")
      {
        $_ = substr($_, 1);
        my @linepart = split(":", $_);
        foreach my $part (@linepart)
        {
          $part =~ s/^\s+|\s+$//g; # Strip unwanted whitespace
        }

        $replay_command = '$self->'.$linepart[0]."(\"";
        my $number_of_parameters = scalar(@linepart);
        for (my $index=1; $index<$number_of_parameters; $index++)
        {
          $replay_command .= $linepart[$index];
          if ($index < ($number_of_parameters-1))
          {
            $replay_command .= ",";
          } 
        }				
        $replay_command .= "\");";
        
        $replay_state = $seek_response;
      }
    }
    elsif ($replay_state == $seek_response)
    {
      if ($_ eq "Response")
      {
        $replay_state = $read_response;
      }
    }
    elsif ($replay_state == $read_response)
    {
      if ($_ eq "End of response")
      {
        # Execute the command we just got the response to...
        $status = 0;
        $status = eval($replay_command);
        if ($self->{'debug_level'} > 1)
        {
          print "Status: $status\n";
          print Dumper($status);
        }
        $replay_state = $get_command;
      }
      else
      {
        if (length($_) > 0)
        {
          push @{$self->{'replay_response'}}, $_;
        }
      }
    }
  }
  close (REPLAYFILE);
}


#*****************************************************************

=head2 Show

When passed the name of an OBD value (e.g. "Engine RPM") in $value_name,
it displays the address of the ECU which returned the value, the name of
the value which was read or the name of one of many parameters returned 
followed by the value and the name of any unit associated with the value.

If an error occurred, a message will be displayed instead.

 $obd->Show($value_name);

This function calls the 'Read' function.
=cut
sub Show
{
	my ($self, $name, $parameter) = @_;

  if ($self->{'debug_level'} > 0) { print "~Show: $name\n"; }
  
  my $response = $self->Read($name, $parameter);

  print "$name:\n";

  if ($response->{'status'} == 0)
  {
    foreach my $result (@{$response->{'results'}})
    {
      print "$result->{'address'} - $result->{'name'}: $result->{'value'}$result->{'unit'}.";
      if (exists($result->{'min_limit'}))
      {
				print " Min: $result->{'min_limit'}$result->{'unit'}.";
			}
      if (exists($result->{'max_limit'}))
      {
				print " Max: $result->{'max_limit'}$result->{'unit'}.";
			}
      print "\n";
    }
  }
  else
  {
    print "$response->{'status_message'}\n";
  }
  print "\n";

  return $response;
}


#*****************************************************************

=head2 Read

When passed the name of an OBD value (e.g. "Engine RPM") in $value_name,
it returns a reference to a structure containing a status flag and any
responses:

 my $response = $obd->Read($value_name);

 if ($response->{'status'} == 0)
 {
  foreach my $result (@{$response->{'results'}})
  {
   print "$result->{'address'} - $result->{'name'}: $result->{'value'} $result->{'unit'}\n"
  }
 }

In the example above, $result->{'address'} contains the address of the
ECU which returned the value, $result->{'name'} contains the name of
the value which was read or the name of one of many parameters returned.
$result->{'value'} and $result->{'unit'} contain the value and the name
of any unit associated with the value.

Error conditions are indicated by a non-zero value of $response->{'status'}
and an error message in $response->{'status_message'} (the default is
'ok' when there is no error).

Error code	Meaning
 0					ok
-1					Unrecognised value
-2					No data returned for value
-3					Unsupported value					
=cut

sub Read
{
	my ($self, $name, $parameter) = @_;

	my $status = 0;
	
	my $results=();
	
	my $id;
	my $value;

	$results->{'status'} = 0;
	$results->{'status_message'} = "ok";

  if ($self->{'debug_level'} > 0) { print "~Read: $name\n"; }

  if (exists($self->{'get'}->{$name}))
  {
		if (exists($self->{'get'}->{$name}->{'available'}) && $self->{'get'}->{$name}->{'available'} == 1)
		{
			my $command = $self->{'get'}->{$name}->{'command'};

			if (substr($name, 0, 12) eq "Freeze frame")
			{
				if (defined($parameter)) 
				{
					$command .= " ";
					$command .= sprintf("%02X", $parameter);
				}
				else
				{
					$command .= " 00";		# Default to freeze frame 0.
				}
			}
			elsif (substr($command, 0, 2) eq "05" && substr($command, 0, 5) ne "05 00")
			{
				if (defined($parameter)) 
				{
					$command .= " ";
					$command .= sprintf("%02X", $parameter);
				}
				else
				{
					$command .= " 01";		# Default to O2Sensor = Bank 1 - Sensor 1.
				}
			}

			$status = $self->Command($command);

			if ($status != 0)
			{
				$results->{'status'} = -2; # No response
				$results->{'status_message'} = "No data returned for: $name";
				return $results;
			}

			if (substr($command, 0, 2) eq "06" && substr($command, 0, 5) ne "06 00" && $self->{'bus_type'} eq "CAN") # Command 6 works differently from 1,2,5,9 etc.
			{
				$self->GetResultsCommand06_CAN(\$results);
			}
			else
			{
				foreach my $result (@{$self->{'get'}->{$name}->{'result'}})
				{
					if ($self->{'debug_level'} > 2) { print Dumper($result); }
					if (exists($result->{'name'}))
					{
						$id = $result->{'name'};  
					}
					else
					{
						$id = $name;
					}

					# Parse the result type...
					my @type = split('_', , $result->{'type'});
					
					my $index = 0;
					my $sign = 0;
					if ($type[0] eq "signed")
					{
						$sign = 1;
						$index = 1;
					}

					if ($type[$index] eq "AT")
					{
						$value = ${$self->{'response'}}[0];

						if (defined($result->{'modifier'}))
						{
							my $statement = '$value' . $result->{'modifier'};
							eval( $statement ); # Allow the use of a regex if required
							if ($self->{'debug_level'} > 2) { print "Statement: $statement\n"; }
						}
						
						if ($self->{'debug_level'} > 1) { print "$id, $value$result->{'unit'}\n"; }
						push @{$results->{'results'}},{name => $id, value => $value, unit => $result->{'unit'}, address => "ELM327"};
					}
					else
					{
						$value = $self->GetResult($type[$index], $type[$index+1]);
						if ($self->{'debug_level'} > 2) { print "GetResult value: $value\n"; }

						my $iteration = 0;
						foreach $value (@{$self->{'command_results'}})
						{
							if ($self->{'debug_level'} > 2) { print "Raw value: $value\n"; }
							
							if ($type[$index] eq "byte" && $type[0] eq "signed")
							{
								my $byte_sign_mask = 128;
								my $byte_value_mask = 127;
								
								$value = ($value & $byte_value_mask) - ($value & $byte_sign_mask);  	
							}
							elsif ($type[$index] eq "word" && $type[0] eq "signed")
							{
								my $word_sign_mask = 32768;
								my $word_value_mask = 32767;
								
								$value = ($value & $word_value_mask) - ($value & $word_sign_mask);  	
							}
							
							if ($type[$index] ne "string")
							{
								if ($self->{'debug_level'} > 2) { print "$value $result->{'modifier'}\n"; }
								$value = eval( "$value $result->{'modifier'}" );
								if ($self->{'debug_level'} > 2) { print "$value\n"; }
								
							}
							else
							{
								my $statement = '$value' . $result->{'modifier'};
								eval( $statement ); # Allow the use of a regex if required
								if ($self->{'debug_level'} > 2) { print "Statement: $statement\n"; }
								
							}
							
							if ($type[$index] eq "bool")
							{
								if ($value != 0) { $value = 1; }			
							}

							if (exists($result->{'alternatives'}))
							{
								foreach my $alternative (@{$result->{'alternatives'}})
								{
									if ($alternative->{'value'} == $value)
									{
										$value = $alternative->{'meaning'};
										last;
									}
								}
							}
							if ($self->{'debug_level'} > 2) { print "$id, $value, $result->{'unit'}\n"; }
							push @{$results->{'results'}},{name => $id, value => $value, unit => $result->{'unit'}, address => ${$self->{'command_addresses'}}[$iteration]};
							$iteration++;
						}
						$self->{'command_results'} = [];
					}
				}
			}
		}
		else
		{
			if ($self->{'debug_level'} > 2) { print "Unsupported name: $name\n"; }
			$results->{'status'} = -3;
			$results->{'status_message'} = "Unsupported name: $name";
		}
	}
	else
	{
    if ($self->{'debug_level'} > 2) { print "Unrecognised name: $name\n"; }
		$results->{'status'} = -1;
		$results->{'status_message'} = "Unrecognised name: $name";
	}
	
  return $results;
}


#*****************************************************************

# Send a command to the ELM, read any response and decode it if
# was an ECU command.
# AT command responses are placed in the $self->{'response'} array.
# Responses to ECU commands are written to the $self->{'results'}
# structure.

sub Command
{
  my ($self, $command) = @_;

  my $status = 0;

  if ($self->{'debug_level'} > 0) { print "~Command: $command\n"; }
  my @command_parts = split(' ', $command);

  $status = $self->WriteCommand("$command");

  $status = $self->ReadResponse();

  if ($command_parts[0] ne "AT" && $status == 0)
  {
    $self->{'last_command'} = hex($command_parts[0]);
    if ($has_sub_command[$self->{'last_command'}])
    {
      $self->{'last_sub_command'} = hex($command_parts[1]);
    }
    else
    {
      $self->{'last_sub_command'} = 0;
    }
      
    $self->DecodeResponse();
  }

  return $status;
}


#*****************************************************************

# Write a command to the serial port

sub WriteCommand
{
  my ($self, $command) = @_;

  my $status = 0;

  if ($self->{'debug_level'} > 0) { print "~WriteCommand: $command\n"; }

  if ($self->{'replay_file'} == 0 && $self->PortOK)
  {
    $command .= "$cr$lf";
    $self->{'port'}->write("$command");
  }

  return $status;
}

#*****************************************************************

# Read the ELM's response from the serial port and put each line 
# into the $self->{'response'} array.
# Turn on debugging for a dump of the response array.

sub ReadResponse
{
  my ($self) = @_;
   
  my $bytes_to_read = 1;
  my $count_in = 0;
  my $string_in = "";
  my $status = 0;
  my $timeout = 4;  # Command 01 04 failed when timeout was 2
  my $line = "";
  $self->{'response'} = ();	# Array of strings, one per line of the response.
  $self->{'response_length'} = 0;

  if ($self->{'debug_level'} > 1) { print "ReadResponse\n"; }

  if ($self->{'replay_file'} == 0 && $self->PortOK)
  {
    do
    {
      ($count_in, $string_in) = $self->{'port'}->read($bytes_to_read);
      if ($count_in == $bytes_to_read && $string_in ne $null)
      {
        $line .= $string_in;
        $self->{'response_length'}++;
      }
      else
      {
        sleep 1;
        $timeout--;
      }
    } while ($count_in == 0 && $timeout>0);

    do
    {
      ($count_in, $string_in) = $self->{'port'}->read($bytes_to_read);
      if ($count_in == $bytes_to_read)
      {
        if ($string_in ne ">" && $string_in ne $null)
        {
          if ($string_in eq $cr)
          {
            if ($line ne "")
            {
              push @{$self->{'response'}}, $line;
              $line = "";
            }
          }
          else
          {
            $line .= $string_in;
            $self->{'response_length'}++;
          }
        }
      }
    } while ($count_in == $bytes_to_read);
  }
  else
  {
    # Load in the lines of the response from the replay file
    while (scalar(@{$self->{'replay_response'}}) > 0)
    {
      $line = shift @{$self->{'replay_response'}};
      push @{$self->{'response'}}, $line;
      $self->{'response_length'} += length($line);
    }
  }

  if ($self->{'response_length'} == 0)
  {
    $status = 1;
  }
  elsif ($self->{'response'}[0] eq "NO DATA")
  {
  	$status = 2;
  }

  if ($self->{'debug_level'} > 0)
  {
    print "Response\n";
    foreach (@{$self->{'response'}})
    {
      print "$_\n";
    }
    print "End of response\n";
  }

  return $status;
}


#*****************************************************************

# Decode the ECU response (in the $self->{'response'} array) and
# write the result to the $self->{'results'} structure.

sub DecodeResponse
{
  my ($self) = @_;

  my $results = {};
  my $command_mask = 63;
  my $line_number = 0;
  my $result_string;

  $self->{'command_addresses'} = [];
  $self->{'command_results'} = [];
  $self->{'number_of_results'} = 0;
   
  if ($self->{'debug_level'} > 1)
  {
    print "\nDecodeResponse\n";
    print "Lines: ".scalar(@{$self->{'response'}})."\n";
  }

  for (keys %{$self->{'results'}})
  {
    delete $self->{'results'}->{$_};
  }
  
  foreach (@{$self->{'response'}})
  {
    my @line_parts = split(' ', $_);
    if (scalar(@line_parts) > 2)
    {
      my $address = shift @line_parts;
      if (length($address) < 3)
      {
        # Not CAN ($address contains the priority byte)
        $self->{'results'}->{$address}->{'format'} = "Other";
        $line_number++;
        my $recipient_address = shift @line_parts;     
        $address = shift @line_parts; # Transmitter address    
        $self->{'results'}->{$address}->{'command'} = (hex(shift @line_parts) & $command_mask);
        if ($self->{'results'}->{$address}->{'command'} == 4) { push(@line_parts, "00"); }
        if ($self->{'results'}->{$address}->{'command'} == $command_mask)
        {
          $self->{'results'}->{$address}->{'command'} = (hex(shift @line_parts));
        }
        if ($has_sub_command[$self->{'results'}->{$address}->{'command'}])
        {     
          $self->{'results'}->{$address}->{'sub_command'} = hex(shift @line_parts);
        }
        else
        {
          $self->{'results'}->{$address}->{'sub_command'} = 0;
        }
				if ($self->{'results'}->{$address}->{'command'} == 2)
				{
					$self->{'results'}->{$address}->{'frame'} = hex(shift @line_parts);
				}
				elsif ($self->{'results'}->{$address}->{'command'} == 5)
				{
					$self->{'results'}->{$address}->{'O2sensor'} = hex(shift @line_parts);
				}
				elsif ($self->{'results'}->{$address}->{'command'} == 6)
				{
					$self->{'results'}->{$address}->{''} = hex(shift @line_parts);
				}
        if (exists($self->{'results'}->{$address}->{'response_length'}))
        {
          $self->{'results'}->{$address}->{'response_length'} += hex(scalar(@line_parts));
        }
        else
        {
          $self->{'results'}->{$address}->{'response_length'} = hex(scalar(@line_parts));
        }
      }
      else
      {
        # CAN
        $self->{'results'}->{$address}->{'format'} = "CAN";
        $line_number = hex(shift @line_parts);
    
        if ($line_number <= 16)
        {
          if ($line_number < 16)
          {
            $self->{'results'}->{$address}->{'response_length'} = $line_number;
          }
          else
          {
            $self->{'results'}->{$address}->{'response_length'} = hex(shift @line_parts);     
          }      
          $self->{'results'}->{$address}->{'command'} = (hex(shift @line_parts) & $command_mask);     
          if ($self->{'results'}->{$address}->{'command'} == 4) { push(@line_parts, "00"); }
          if ($self->{'results'}->{$address}->{'command'} == $command_mask)
          {
            $self->{'results'}->{$address}->{'command'} = (hex(shift @line_parts));
          }
          if ($has_sub_command[$self->{'results'}->{$address}->{'command'}])
          {     
            $self->{'results'}->{$address}->{'sub_command'} = hex(shift @line_parts);
          }
          else
          {
            $self->{'results'}->{$address}->{'sub_command'} = 0;
            if ($self->{'results'}->{$address}->{'command'} == 3 ||
                $self->{'results'}->{$address}->{'command'} == 7 ||
                $self->{'results'}->{$address}->{'command'} == 10
               )
            {
              $self->{'results'}->{$address}->{'number_of_DTCs'} = hex(shift @line_parts);
            }
          }
          if ($self->{'results'}->{$address}->{'command'} == 2)
          {
						$self->{'results'}->{$address}->{'frame'} = hex(shift @line_parts);
					}
        }     
      }
      $self->{'bus_type'} = $self->{'results'}->{$address}->{'format'};  
      $results->{$address}->{$line_number} = join(" ", @line_parts);
    }
  }

  if ($self->{'debug_level'} > 1)
  {
    print "Decoded results:\n";  
    print Dumper($results);
  }

  foreach my $address (sort keys %{$results})
  {
    $result_string = "";
    foreach my $line (sort keys %{$results->{$address}})
    {
      $result_string .= "$results->{$address}->{$line} "; 
    }
    @{$self->{'results'}->{$address}->{'result'}} = split(' ', $result_string);


    #Now turn the hex byte strings into numbers...
    foreach (@{$self->{'results'}->{$address}->{'result'}})
    {
      $_ = hex($_);
    }
  }

  if ($self->{'debug_level'} > 1)
  {
    print "\nFully decoded results:\n";  
    print Dumper($self->{'results'});
  }
  
}


#*****************************************************************

# Returns a value from the last set of results from the ELM/ECU

# $type can be one of the following:
# bool (1 bit), byte (8 bit), word (16 bit), dword (32 bit) or string.
# $number is the zero-based index into the array of results and takes
# the type into account such that $number=0 returns the first byte,
# word or dword and $number=1, returns the second.
# Booleans are treated the same as bytes and require individual bits
# to be extracted separately.
# For strings, $number is the offset of the start of the string.

sub GetResult
{
	my ($self, $type, $number) = @_;

  my $result;
  $self->{'number_of_results'} = 0;

  if (!defined($number)) { $number = 0; }
  
  if ($self->{'debug_level'} > 1) { print "GetResult: $type: $number\n"; }

  foreach my $address (sort keys %{$self->{'results'}})
  {
    if ($self->{'results'}->{$address}->{'command'} == $self->{'last_command'} && 
    $self->{'results'}->{$address}->{'sub_command'} == $self->{'last_sub_command'})
    {
      if ($type eq "bool")
      {
        $result = ${$self->{'results'}->{$address}->{'result'}}[$number];
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
      elsif ($type eq "byte")
      {
        $result = ${$self->{'results'}->{$address}->{'result'}}[$number];
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
      elsif ($type eq "word")
      {
        $result = ((${$self->{'results'}->{$address}->{'result'}}[$number*2] * 256) + ${$self->{'results'}->{$address}->{'result'}}[($number*2)+1] );
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
      elsif ($type eq "dword")
      {
        $result = ((${$self->{'results'}->{$address}->{'result'}}[$number*4] * 16777216) + ${$self->{'results'}->{$address}->{'result'}}[($number*4)+1] * 65536);
        $result += ((${$self->{'results'}->{$address}->{'result'}}[($number*4)+2] * 256) + ${$self->{'results'}->{$address}->{'result'}}[($number*4)+3] );
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
      elsif ($type eq "string")
      {
        $result = "";
        foreach (@{$self->{'results'}->{$address}->{'result'}})
        {
          if ($number > 0)
          {
            $number--;
          }
          else
          {
            if ($_ > 32 && $_ < 127)  # Ignore non-printable characters
            {
              $result .= chr($_);
            }
          }
        }
        push @{$self->{'command_addresses'}}, $address;
        push @{$self->{'command_results'}}, $result;
        $self->{'number_of_results'}++;
      }
    }
    else
    {
      $result = 0;
    }
  }
  
  return $result;
}


#*****************************************************************

sub GetResultsCommand06_CAN
{
	my ($self, $results_ref) = @_;

	my $results = $$results_ref;

  foreach my $address (sort keys %{$self->{'results'}})
  {
		my $index = 1;
		my $number_of_bytes = scalar(@{$self->{'results'}->{$address}->{'result'}});
		do
		{
			my $obdmid_id = ${$self->{'results'}->{$address}->{'result'}}[$index];
			my $sdt_id = ${$self->{'results'}->{$address}->{'result'}}[$index+1];
			my $uas_id = ${$self->{'results'}->{$address}->{'result'}}[$index+2];

			my $test_value = ${$self->{'results'}->{$address}->{'result'}}[$index+3] * 256;
			$test_value += ${$self->{'results'}->{$address}->{'result'}}[$index+4];

			my $min_test_limit = ${$self->{'results'}->{$address}->{'result'}}[$index+5] * 256;
			$min_test_limit += ${$self->{'results'}->{$address}->{'result'}}[$index+6];

			my $max_test_limit = ${$self->{'results'}->{$address}->{'result'}}[$index+7] * 256;
			$max_test_limit += ${$self->{'results'}->{$address}->{'result'}}[$index+8];

			my $test_name = "Unrecognised test Id";

			if (exists($self->{'Standardized_Test_IDs'}->{$sdt_id}))
			{
				$test_name = $self->{'Standardized_Test_IDs'}->{$sdt_id}->{'name'};
			}

			my $unit = "unknown";

			if ($uas_id >= 128)
			{
				my $word_sign_mask = 32768;
				my $word_value_mask = 32767;
				
				$test_value = ($test_value & $word_value_mask) - ($test_value & $word_sign_mask);  	
				$min_test_limit = ($min_test_limit & $word_value_mask) - ($min_test_limit & $word_sign_mask);  	
				$max_test_limit = ($max_test_limit & $word_value_mask) - ($max_test_limit & $word_sign_mask);  	
			}
			
			if (exists($self->{'unit_and_scaling_identifiers'}->{$uas_id}))
			{
				$unit = $self->{'unit_and_scaling_identifiers'}->{$uas_id}->{'unit'};

				$test_value = eval( "$test_value $self->{'unit_and_scaling_identifiers'}->{$uas_id}->{'modifier'}" );
				$min_test_limit = eval( "$min_test_limit $self->{'unit_and_scaling_identifiers'}->{$uas_id}->{'modifier'}" );
				$max_test_limit = eval( "$max_test_limit $self->{'unit_and_scaling_identifiers'}->{$uas_id}->{'modifier'}" );
			}
			
			push @{$results->{'results'}},{address => $address, name => $test_name, value => $test_value, max_limit => $max_test_limit, min_limit => $min_test_limit, unit => $unit};

			$index += 9;
		} while($index < $number_of_bytes);
	}

	if ($self->{'debug_level'} > 2)
	{
		print "Command 06 - results:\n";
		print Dumper($self->{'results'});
	}
}


#*****************************************************************

=head2 ShowTroubleCodes

Display any trouble codes on the console:

 $obd->ShowTroubleCodes();
=cut

sub ShowTroubleCodes
{
	my ($self) = @_;

  my $malfunction_indicator_lamp_mask = 128;
  my $number_of_codes_mask = 127;

  if ($self->{'debug_level'} > 0) { print "ShowTroubleCodes\n"; }

  $self->Command("01 01");  # Get number of trouble codes

  my $number_of_codes = $self->GetResult("byte");
  my $malfunction_indicator_lamp_state = $number_of_codes & $malfunction_indicator_lamp_mask;
  $number_of_codes &= $number_of_codes_mask;
   
  if ($number_of_codes > 0)
  {
    $self->DisplayTroubleCodes();
  }
  else
  {
    print "No trouble codes found.\n";
  }
}


#*****************************************************************

# This function is called by ShowTroubleCodes. It displays any
# trouble codes on the console.

sub DisplayTroubleCodes
{
	my ($self) = @_;

  my @codes = ("P0","P1","P2","P3","C0","C1","C2","C3","B0","B1","B2","B3","U0","U1","U2","U3");

  my $code_prefix_mask = 61440;
  my $code_mask = 4095;
  my $number_of_codes_mask = 127;
  my $result = 0;
  $self->{'trouble_codes'} = [];

  if ($self->{'debug_level'} > 0) { print "~DisplayTroubleCodes\n"; }

  $self->Command("03");  # Get trouble codes
  
  foreach my $address (sort keys %{$self->{'results'}})
  {
    if ($self->{'debug_level'} > 1)
    {
      print "Message Type: $self->{'results'}->{$address}->{'format'}\n";
      print "$address\n";
    }
    if ($self->{'results'}->{$address}->{'command'} == $self->{'last_command'})
    {
      my $index = 0;
      do
      {
        $result = ((${$self->{'results'}->{$address}->{'result'}}[$index] * 256) + ${$self->{'results'}->{$address}->{'result'}}[$index+1] );
        if ($result !=0)
        {
          push @{$self->{'command_addresses'}}, $address;
          push @{$self->{'command_results'}}, $result;
          $self->{'number_of_results'}++;
        }
        $index += 2;
      } while ($result != 0);
    }
  }

  if ($self->{'debug_level'} > 1)
  {
    print "Error code(s): @{$self->{'command_results'}}\n";
  }
  
  foreach my $code (@{$self->{'command_results'}})
  {
    my $code_prefix = ($code & $code_prefix_mask) >> 12;
    $code &= $code_mask;
    $code = sprintf("%03X", $code);
    my $decoded_code = "$codes[$code_prefix]$code";
    push @{$self->{'trouble_codes'}} , $decoded_code;
          
    if ($self->{'debug_level'} > 1)
    {
      print "Code prefix: $code_prefix, Code: $code, Decoded: $decoded_code\n";
    }
  }
}

#*****************************************************************

=head2 ClearTroubleCodes

Clear any Trouble Codes and sensor data:

 $obd->ClearTroubleCodes();

Note that clearing the sensor data will cause the vehicle to run on
default values until it has recalibrated itself. This may affect
performance and fuel economy.

The ISO specification also insists that any user interface which
invokes this function should display an "are you sure?" message
to the user before calling it.

=cut

sub ClearTroubleCodes
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~ClearTroubleCodes\n"; }
  $self->Command("04");  # Clear Trouble Codes and sensor data

  my $result = $self->GetResult("byte");
  return $result;  # Returns 0 if codes have been cleared or error code.
}


#*****************************************************************

=head2 GetVIN

Returns a string containing the Vehicle Identification Number (VIN)

 my $vin = $obd->GetVIN();

This value can also be obtained using the Show and Read commands:

 $obd->Show("Vehicle Identification Number");
=cut

sub GetVIN
{
	my ($self) = @_;
  
  if ($self->{'debug_level'} > 0) { print "~GetVIN:\n"; }

  $self->Command("09 02");  # Get VIN

  my $vin = $self->GetResult("string", 1);

  return $vin;
}


#*****************************************************************

=head2 GetELM

Returns a string containing the type of the ELM module, e.g.:
"ELM327 v1.3a"

 my $elm = $obd->GetELM();

This value can also be obtained using the Show and Read commands:

 $obd->Read("ELM identity");
=cut

sub GetELM
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~GetELM:\n"; }

  $self->Command("AT I");  # Read Identity
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 GetVoltage

Returns a string containing the Voltage measured by the ELM module 
(e.g. "13.9V"):

 my $Voltage = $obd->GetVoltage();

The number and unit can be obtained separately by calling:

 my $response = $obd->read("Input Voltage");
=cut

sub GetVoltage
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~GetVoltage:\n"; }
  
  $self->Command("AT RV");  # Read Voltage
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 CalibrateVoltage

Changes the calibration value used by the ELM module. The value
$Voltage is a string containing a fixed point value of the form:
xx.xx, e.g "11.99", "12.01" etc.

 $obd->CalibrateVoltage($Voltage);
=cut

sub CalibrateVoltage
{
	my ($self, $Voltage) = @_;

  if ($self->{'debug_level'} > 0) { print "~CalibrateVoltage: $Voltage\n"; }
  
  $self->Command("AT CV $Voltage");  # Calibrate Voltage
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 ResetVoltage

Resets the ELM module's Voltage calibration to the factory setting:

 $obd->ResetVoltage();
=cut

sub ResetVoltage
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~ResetVoltage:\n"; }

  $self->Command("AT CV 0000");  # Reset Voltage to factory setting
  return ${$self->{'response'}}[0];
}


#*****************************************************************

# Returns ignition state (ON or OFF)

sub GetIgnition
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~GetIgnition:\n"; }
  
  $self->Command("AT IGN");  # Read ignition state (ON or OFF)
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 GetStoredDataByte

Returns the value previously stored in the ELM module's non-volatile
storage area:

 my $byte_value = $obd->WriteStoredDataByte();

This value can also be read using:

 $obd->Read("Stored data byte");
=cut

sub GetStoredDataByte
{
	my ($self) = @_;

  if ($self->{'debug_level'} > 0) { print "~GetStoredDataByte:\n"; }
  
  $self->Command("AT RD");  # Read the stored data
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head2 WriteStoredDataByte

Writes $byte_value to the ELM module's non-volatile storage area.

 $obd->WriteStoredDataByte($byte_value);
=cut

sub WriteStoredDataByte
{
	my ($self, $value) = @_;

  if ($self->{'debug_level'} > 0) { print "~WriteStoredDataByte: $value\n"; }
  
  $self->Command("AT SD $value");  # Save Data byte value
  return ${$self->{'response'}}[0];
}


#*****************************************************************

=head1 AUTHOR

Alister Perrott, C<< <aperrott at cpan.org> >>

=head1 BUGS

Please report any bugs or feature requests to C<bug-device-elm327 at rt.cpan.org>, or through
the web interface at L<http://rt.cpan.org/NoAuth/ReportBug.html?Queue=Device-ELM327>. 
I will be notified, and then you'll automatically be notified of progress on your bug as I make changes.

Please also include a debug trace showing the error. This can be done by setting $debug_level in the constructor to 1 and piping the output to a file.
e.g. perl myOBD.pl&>trace.txt


=head1 SUPPORT

You can find documentation for this module with the perldoc command.

  perldoc Device::ELM327


You can also look for information at:

=over 5

=item * RT: CPAN's request tracker

L<http://rt.cpan.org/NoAuth/Bugs.html?Dist=Device-ELM327>

=item * AnnoCPAN: Annotated CPAN documentation

L<http://annocpan.org/dist/Device-ELM327>

=item * CPAN Ratings

L<http://cpanratings.perl.org/d/Device-ELM327>

=item * Search CPAN

L<http://search.cpan.org/dist/Device-ELM327/>

=item * My ELM327 page

L<http://gts-ltd.co.uk/ELM327.php>

=back


=head1 ACKNOWLEDGEMENTS

Many thanks to:
  The authors of Win32::SerialPort and Device::SerialPort.
  Larry Wall and all the other people who have worked on Perl.
  ELM Electronics for creating the ELM327 module.
  Everyone involved with CPAN.

=head1 LICENSE AND COPYRIGHT

Copyright 2012 Alister Perrott.

This program is free software; you can redistribute it and/or modify it
under the terms of either: the GNU General Public License as published
by the Free Software Foundation; or the Artistic License.

See http://dev.perl.org/licenses/ for more information.


=cut

#********************************************************************
1; # End of Device::ELM327 - Return success to require/use statement
#********************************************************************
