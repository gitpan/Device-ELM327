use strict;
use warnings;
use ExtUtils::MakeMaker;

my $makemaker_version = $ExtUtils::MakeMaker::VERSION;
$makemaker_version=~ s/_//;

WriteMakefile(
    NAME                => 'Device::ELM327',
    AUTHOR              => q{Alister Perrott <aperrott@cpan.org>},
    VERSION_FROM        => 'lib/Device/ELM327.pm',
    ABSTRACT_FROM       => 'lib/Device/ELM327.pm',
    ($makemaker_version >= 6.3002
      ? ('LICENSE'=> 'perl')
      : ()),
    PL_FILES            => {},
    PREREQ_PM => {
        'Test::More' => 0,
    },
    dist                => { COMPRESS => 'gzip -9f', SUFFIX => 'gz', },
    clean               => { FILES => 'Device-ELM327-*' },
);
