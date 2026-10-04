#!/usr/bin/env bash
# Pure parser for systemctl show -P ExecStopPost; sourcing performs no actions.
unit_post_succeeded() {
    # systemd prints a successful exit as 0, 0/0, or 0/SUCCESS.
    local pattern='\{[^{}]*[[:space:];]code=exited[[:space:]]*;[[:space:]]*status=0(/(0|SUCCESS))?[[:space:]]*\}'
    [[ $1 =~ $pattern ]]
}

# systemd may clear the per-command exit details once the normal service is
# inactive. Accept that exact cleared form only with an inactive unit, last-known
# Result=success, and the expected non-ignored recovery command configured.
# For a unit already inactive on entry, this cannot prove a particular previous
# stop completed or establish BIOS fan ownership. The caller must also check
# pending recovery state before replacing the old binary.
normal_service_stop_verified() {
    local post=$1 unit_state=$2 result=$3
    [[ $unit_state == inactive && $result == success ]] || return 1
    [[ $post =~ ^\{[^{}]*\}$ ]] || return 1
    [[ $post == '{ path=/usr/bin/nzxt-cam-hwd ; argv[]=/usr/bin/nzxt-cam-hwd recover-service ; ignore_errors=no ; '* ]] || return 1
    unit_post_succeeded "$post" && return 0
    [[ $post == *' ; start_time=[n/a] ; stop_time=[n/a] ; pid=0 ; code=(null) ; status=0/0 }' ]]
}
