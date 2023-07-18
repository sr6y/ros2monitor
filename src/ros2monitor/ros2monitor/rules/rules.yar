rule Example
{
    strings:
        $string = "Publishing"

    condition:
        $string
}

rule PayloadPHP
{
	strings:
	    $payload = "<?php"
	condition:
	    $payload
}

rule PayloadJS
{
	strings:
	    $payload = "<script>"
	condition:
	    $payload
}