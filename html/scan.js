document.addEventListener("DOMContentLoaded", init, false);

let aatServoTestNeutralUs = 1520;
let latestConfigData = null;
let refreshAatServoSetupPanel = null;

function _(el) {
    return document.getElementById(el);
}

function init() {
    initAat();
    initAatServoTest();
    initMavLink();

    // sends XMLHttpRequest, so do it last
    initOptions();
}

function initAatServoTest() {
    const panel = _('aat-servo-test');
    if (!panel)
        return;

    const slider = _('aat-servo-pulse');
    const value = _('aat-servo-pulse-value');
    const status = _('aat-servo-status');
    const outputStatus = _('aat-servo-output');
    const driverStatus = _('aat-servo-driver');
    const rawCompassStatus = _('aat-servo-raw');
    const hmcStatus = _('aat-servo-hmc');
    const compassStatus = _('aat-servo-compass');
    const rateStatus = _('aat-servo-rate');
    const pulseCalibrationButton = _('aat-servo-pulse-calibrate');
    const pulseCalibrationStopButton = _('aat-servo-pulse-stop');
    const pulseCalibrationStatus = _('aat-servo-pulse-calibration');
    const mountCheckStartButton = _('aat-servo-mount-check-start');
    const mountCheckValidateButton = _('aat-servo-mount-check-validate');
    const mountCheckResetButton = _('aat-servo-mount-check-reset');
    const mountReverseButton = _('aat-servo-mount-reverse');
    const mountCheckStatus = _('aat-servo-mount-check-status');
    const referenceStatus = _('aat-servo-reference');
    const sweepStatus = _('aat-servo-sweep');
    const sweepHint = _('aat-servo-sweep-hint');
    const sweepSamples = _('aat-servo-samples');
    const sweepDownloadButton = _('aat-servo-sweep-download');
    const sweepExportStatus = _('aat-servo-sweep-export-status');
    const fitStatus = _('aat-servo-fit');
    const fitQuality = _('aat-servo-fit-quality');
    const neutralButton = _('aat-servo-neutral');
    const sweepStartButton = _('aat-servo-sweep-start');
    const sweepStopButton = _('aat-servo-sweep-stop');
    const calibrationSaveButton = _('aat-servo-calibration-save');
    const northAlignButton = _('aat-servo-north-align');
    const northAlignStatus = _('aat-servo-north-status');
    let servoTestSweepActive = false;
    let servoTestRefreshTimer = null;
    let setupStatusRequestPending = false;
    let sweepExportInProgress = false;
    let sweepExported = false;
    let latestServoData = null;
    let mountCheckStartHeading = null;
    let mountCheckPassed = false;
    let mountReverseAvailable = false;
    let mountRequestPending = false;
    let pulseCalibrationActive = false;

    function isNeutralCompassCheckReady(data) {
        return data && data.setup_active && data.saved_calibration_valid
            && data.calibrated_heading_valid && data.compass_config_a_matches_requested
            && data.compass_status_after_trigger_valid && !data.compass_data_locked_after_trigger
            && data.pulse_us === data.neutral_us && data.output_pulse_us === data.neutral_us
            && !data.reference_pressed && !data.reference_raw_pressed
            && data.sweep_state !== 'armed' && data.sweep_state !== 'sweeping';
    }

    function renderPulse(pulseUs) {
        slider.value = pulseUs;
        value.value = `${pulseUs} us`;
        status.textContent = pulseUs === aatServoTestNeutralUs ? 'Neutral' : 'Pulse command active';
    }

    async function sendPulse(pulseUs) {
        if (servoTestSweepActive) {
            if (pulseUs === aatServoTestNeutralUs)
                return sendSweepCommand('stop');
            return;
        }
        try {
            const response = await fetch('/aatservo', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: new URLSearchParams({ pulse: pulseUs })
            });
            if (!response.ok)
                throw new Error(await response.text());
            const data = await response.json();
            renderPulse(data.pulse_us);
            outputStatus.textContent = `Servo output: ${data.output_pulse_us} us`;
            driverStatus.textContent = data.servo_attached
                ? `Servo driver: attached, ${data.library_pulse_us} us`
                : 'Servo driver: not attached';
            rawCompassStatus.textContent = `Raw compass X/Y: ${data.raw_x} / ${data.raw_y}; ${data.compass_sample_count} samples, ${data.compass_sample_age_ms} ms old`;
            hmcStatus.textContent = `HMC ${data.compass_config_a}/${data.compass_config_b}/${data.compass_mode}; polls ${data.compass_poll_count}; pre RDY/LOCK ${data.compass_data_ready}/${data.compass_data_locked}; post ${data.compass_data_ready_after_read}/${data.compass_data_locked_after_read}`;
            renderCompass(data);
        } catch (error) {
            status.textContent = error.message;
            await refreshCompass();
        }
    }

    function renderCompass(data) {
        const firstCompassUpdate = latestServoData === null;
        latestServoData = data;
        pulseCalibrationActive = data.pulse_calibration_active === true;
        if (firstCompassUpdate && data.mounting_verification_pending) {
            mountCheckStatus.textContent = 'Correction saved. Capture a new starting heading and repeat the clockwise turn.';
        } else if (firstCompassUpdate && data.saved_calibration_valid && data.mounting_reversed) {
            mountCheckPassed = true;
            mountCheckStatus.textContent = 'Mounting response: reversed direction verified and saved.';
        } else if (firstCompassUpdate && data.saved_calibration_valid && data.saved_north_aligned) {
            mountCheckPassed = true;
            mountCheckStatus.textContent = 'Mounting response: previously commissioned with saved fit and north alignment';
        }
        const sweepArmed = data.sweep_state === 'armed';
        const sweepActive = sweepArmed || data.sweep_state === 'sweeping';
        const compassConfigMatches = data.compass_config_a_matches_requested;
        const triggerStatusValid = data.compass_status_after_trigger_valid;
        const triggerLockClear = triggerStatusValid && !data.compass_data_locked_after_trigger;
        servoTestSweepActive = sweepActive;
        const mountingCheckReady = !mountRequestPending && !pulseCalibrationActive && isNeutralCompassCheckReady(data);
        mountCheckStartButton.disabled = !mountingCheckReady || mountCheckStartHeading !== null;
        mountCheckValidateButton.disabled = !mountingCheckReady || mountCheckStartHeading === null;
        mountReverseButton.hidden = !mountReverseAvailable || data.mounting_reversed;
        mountReverseButton.disabled = !mountingCheckReady || mountReverseButton.hidden;
        mountCheckResetButton.disabled = mountRequestPending;
        slider.min = data.min_us;
        slider.max = data.max_us;
        slider.disabled = sweepActive || pulseCalibrationActive || !data.manual_pulse_allowed;
        neutralButton.disabled = !data.setup_active;
        renderPulse(data.pulse_us);
        outputStatus.textContent = `Servo output: ${data.output_pulse_us} us`;
        driverStatus.textContent = data.servo_attached
            ? `Servo driver: attached, ${data.library_pulse_us} us`
            : 'Servo driver: not attached';
        rawCompassStatus.textContent = `Raw compass X/Y: ${data.raw_x} / ${data.raw_y}; ${data.compass_sample_count} samples, ${data.compass_sample_age_ms} ms old`;
        hmcStatus.textContent = `HMC ${data.compass_config_a}/${data.compass_config_b}/${data.compass_mode}; polls ${data.compass_poll_count}; pre RDY/LOCK ${data.compass_data_ready}/${data.compass_data_locked}; post ${data.compass_data_ready_after_read}/${data.compass_data_locked_after_read}`;
        if (sweepArmed)
            status.textContent = 'Owner calibration armed';
        else if (data.sweep_state === 'sweeping')
            status.textContent = `Sweep pulse active: ${data.output_pulse_us} us`;
        referenceStatus.textContent = data.reference_pressed
            ? 'GPIO32: pressed'
            : data.reference_raw_pressed ? 'GPIO32: pressed, debouncing' : 'GPIO32: released';
        sweepStatus.textContent = `Sweep: ${data.sweep_state}`;
        sweepSamples.textContent = `Samples: ${data.sweep_samples}`;
        sweepDownloadButton.disabled = sweepExportInProgress
            || data.sweep_state !== 'complete' || data.sweep_samples === 0;
        sweepStartButton.disabled = sweepActive || pulseCalibrationActive || data.reference_pressed || data.reference_raw_pressed
            || !data.compass_detected
            || !data.heading_valid || !compassConfigMatches || !triggerLockClear;
        sweepStopButton.disabled = !sweepActive && !pulseCalibrationActive;
        calibrationSaveButton.disabled = pulseCalibrationActive || !data.calibration_ready || data.calibration_saved;
        const actuatorNeutral = data.pulse_us === data.neutral_us
            && data.output_pulse_us === data.neutral_us;
        pulseCalibrationButton.disabled = !data.pulse_calibration_supported
            || pulseCalibrationActive || !data.setup_active
            || !data.saved_calibration_valid || !data.saved_north_aligned
            || !data.manual_pulse_allowed || data.pulse_us !== data.neutral_us
            || data.output_pulse_us !== data.neutral_us;
        pulseCalibrationButton.textContent = pulseCalibrationActive
            ? 'Pulse calibration running' : 'Run automatic pulse calibration';
        const savedPulseSummary = `Pulse calibration: saved left ${data.left_start_pulse_us} us, right ${data.right_start_pulse_us} us`;
        if (pulseCalibrationActive) {
            const direction = data.pulse_calibration_state === 1 ? 'left' : 'right';
            pulseCalibrationStatus.textContent = data.pulse_calibration_settling
                ? `Pulse calibration: settling at neutral before ${direction} stage`
                : `Pulse calibration: ${direction} at ${data.pulse_calibration_candidate_us} us, ${data.pulse_calibration_samples}/3 direction windows, net ${(data.pulse_calibration_displacement_mdeg / 1000).toFixed(1)} deg (more than 6 deg required)`;
        } else if (data.pulse_calibration_state === 3) {
            pulseCalibrationStatus.textContent = savedPulseSummary;
        } else if (data.pulse_calibration_state === 4) {
            const faults = ['none', 'invalid configuration', 'safety gate lost',
                'no reliable motion within maximum offset', 'load did not settle',
                'wrong-direction motion', 'stage travel exceeded', 'total timeout',
                'unexpected or delayed output'];
            pulseCalibrationStatus.textContent =
                `Pulse calibration: ${faults[data.pulse_calibration_fault] || 'failed'}; pan returned to neutral`;
        } else if (!data.pulse_calibration_supported) {
            pulseCalibrationStatus.textContent = 'Automatic pulse calibration requires CompassPan firmware';
        } else if (data.pulse_calibration_valid) {
            pulseCalibrationStatus.textContent = savedPulseSummary;
        } else {
            pulseCalibrationStatus.textContent = 'Pulse calibration: not running; no valid saved pulses';
        }
        northAlignButton.disabled = sweepActive || pulseCalibrationActive || !data.saved_calibration_valid
            || !data.calibrated_heading_valid || !compassConfigMatches || !triggerLockClear
            || data.reference_pressed || data.reference_raw_pressed || !actuatorNeutral
            || data.mounting_verification_pending || (!data.saved_north_aligned && !mountCheckPassed)
            || data.saved_north_aligned;
        northAlignButton.textContent = data.saved_north_aligned
            ? 'True north saved'
            : 'Press when device is true north';
        northAlignStatus.textContent = data.saved_north_aligned
            ? 'True-north calibration: saved'
            : 'True-north calibration: required';
        if (sweepArmed)
            sweepHint.textContent = 'Hold GPIO32 at the reference mark to start rotation.';
        else if (data.sweep_state === 'sweeping')
            sweepHint.textContent = 'Keep GPIO32 held through one full turn. Release at the reference mark to finish, or use Stop to abort.';
        else if (data.calibration_saved && !data.manual_pulse_allowed)
            sweepHint.textContent = 'Compass fit saved. Check compass status, then press and release GPIO32 at the mechanical mark to unlock direction checks.';
        else if (data.calibration_saved)
            sweepHint.textContent = 'Compass fit saved. Direction checks are unlocked; reboot afterward to begin startup alignment.';
        else if (data.sweep_state === 'fault')
            sweepHint.textContent = data.fit_attempted
                ? `Fit rejected (${data.fit_failure}): ${data.fit_samples} samples, X span ${data.span_x}, Y span ${data.span_y}, ${data.covered_sectors}/12 sectors.`
                : `Sweep stopped (${data.sweep_fault_reason || 'unknown'}). Neutral has been applied.`;
        else if (!data.compass_detected || !data.heading_valid)
            sweepHint.textContent = 'Waiting for a fresh compass sample.';
        else if (!compassConfigMatches)
            sweepHint.textContent = 'Compass setup is not ready: configuration does not match this firmware.';
        else if (!triggerStatusValid)
            sweepHint.textContent = 'Waiting for a valid post-trigger compass status.';
        else if (!triggerLockClear)
            sweepHint.textContent = 'Waiting for compass LOCK to clear.';
        else if (data.reference_pressed || data.reference_raw_pressed)
            sweepHint.textContent = 'Release GPIO32 before arming owner calibration.';
        else if (!data.manual_pulse_allowed)
            sweepHint.textContent = 'Manual pulse locked: verify the pan is within 90 degrees of the captured reference.';
        else if (data.sweep_state === 'complete')
            sweepHint.textContent = data.calibration_ready
                ? 'Owner fit captured. Review the fit before saving.'
                : 'Sweep complete, but the fit needs more coverage.';
        else if (!data.manual_pulse_allowed)
            sweepHint.textContent = 'Manual pulse locked: verify the pan is within 90 degrees of the captured reference.';
        else
            sweepHint.textContent = 'Arm owner calibration, then hold GPIO32 to start rotation.';
        if (!data.compass_detected) {
            compassStatus.textContent = 'Compass not detected';
            rateStatus.textContent = 'Relative turn rate: --';
            fitStatus.textContent = data.saved_calibration_valid ? 'Compass fit: saved' : 'Compass fit: not calibrated';
            fitQuality.textContent = 'Fit quality unavailable: compass not detected';
            return;
        }
        compassStatus.textContent = data.heading_valid
            ? `Raw heading: ${(data.heading_mdeg / 1000).toFixed(1)} deg`
            : 'Compass sample unavailable';
        rateStatus.textContent = data.rate_valid
            ? `Relative turn rate: ${(data.rate_mdeg_s / 1000).toFixed(1)} deg/s`
            : 'Relative turn rate: waiting for samples';
        if (data.calibrated_rate_valid)
            rateStatus.textContent += `; calibrated ${(data.calibrated_rate_mdeg_s / 1000).toFixed(1)} deg/s`;
        fitStatus.textContent = data.fit_attempted && !data.calibration_ready
            ? `Fit rejected (${data.fit_failure}): ${data.covered_sectors}/12 sectors, return mismatch ${data.return_closure_error_deg} deg`
            : data.calibration_ready
            ? `${data.calibration_saved ? 'Fit saved' : 'Fit ready'}: offsets ${data.offset_x.toFixed(1)}, ${data.offset_y.toFixed(1)}; scales ${data.scale_x.toFixed(2)}, ${data.scale_y.toFixed(2)}`
            : (data.saved_calibration_valid ? 'Compass fit: saved' : 'Compass fit: not calibrated');
        const hasFitQualityMetrics = data.fit_attempted && data.fit_samples >= 36
            && Number.isFinite(data.field_magnitude_min)
            && Number.isFinite(data.field_magnitude_max) && data.field_magnitude_max > 0;
        const fieldMagnitudeRatio = Number.isFinite(data.field_magnitude_ratio)
            ? `${data.field_magnitude_ratio.toFixed(2)}x` : 'undefined';
        fitQuality.textContent = hasFitQualityMetrics
            ? `Coverage ${data.covered_sectors}/12; return mismatch ${data.return_closure_error_deg} deg; field magnitude ${data.field_magnitude_min.toFixed(0)}-${data.field_magnitude_max.toFixed(0)} (${fieldMagnitudeRatio})`
                + (data.field_magnitude_warning ? ' - check for magnetic interference' : '')
            : data.fit_attempted
            ? 'Fit quality metrics unavailable: too few samples.'
            : data.saved_calibration_valid
            ? 'Saved fit; quality details are available after a new owner calibration.'
            : 'Fit quality: run an owner calibration';
    }

    async function refreshCompass() {
        try {
            const response = await fetch('/aatservo', { cache: 'no-store' });
            if (!response.ok)
                throw new Error('Setup status unavailable');
            const data = await response.json();
            if (!data.setup_active) {
                panel.style.display = 'none';
                if (servoTestRefreshTimer) {
                    clearInterval(servoTestRefreshTimer);
                    servoTestRefreshTimer = null;
                }
                return;
            }
            panel.style.display = 'block';
            renderCompass(data);
        } catch (error) {
            if (panel.style.display !== 'none')
                status.textContent = `Setup status unavailable: ${error.message}`;
        }
    }

    async function refreshSetupPanel() {
        if (setupStatusRequestPending)
            return;
        setupStatusRequestPending = true;
        try {
            const response = await fetch('/aatservo', { cache: 'no-store' });
            if (!response.ok)
                throw new Error('Setup endpoint unavailable');
            const data = await response.json();
            panel.style.display = data.setup_active ? 'block' : 'none';
            if (!data.setup_active) {
                if (servoTestRefreshTimer) {
                    clearInterval(servoTestRefreshTimer);
                    servoTestRefreshTimer = null;
                }
                return;
            }
            aatServoTestNeutralUs = data.neutral_us;
            slider.min = data.min_us;
            slider.max = data.max_us;
            renderPulse(data.pulse_us);
            renderCompass(data);
            if (!servoTestRefreshTimer)
                servoTestRefreshTimer = setInterval(refreshCompass, 200);
        } catch (error) {
            panel.style.display = 'none';
            status.textContent = `Setup status unavailable: ${error.message}`;
        } finally {
            setupStatusRequestPending = false;
        }
    }

    refreshAatServoSetupPanel = refreshSetupPanel;

    slider.addEventListener('input', () => sendPulse(Number(slider.value)));
    neutralButton.addEventListener('click', () => sendSweepCommand('stop'));
    pulseCalibrationStopButton.addEventListener('click', () => sendSweepCommand('stop'));
    sweepStartButton.addEventListener('click', () => sendSweepCommand('owner'));
    sweepStopButton.addEventListener('click', () => sendSweepCommand('stop'));
    sweepDownloadButton.addEventListener('click', downloadSweepCsv);
    calibrationSaveButton.addEventListener('click', saveCompassCalibration);
    northAlignButton.addEventListener('click', saveTrueNorthReference);
    pulseCalibrationButton.addEventListener('click', async () => {
        if (!window.confirm('Servo power must be connected. Secure the loaded pan and clear at least 90 degrees of travel in both directions. Firmware will apply guarded staged pulses automatically. Continue?'))
            return;
        try {
            const response = await fetch('/aatservo', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: new URLSearchParams({ calibrate_pulses: 1 })
            });
            if (!response.ok)
                throw new Error(await response.text());
            renderCompass(await response.json());
        } catch (error) {
            pulseCalibrationStatus.textContent = `Pulse calibration: ${error.message}`;
        }
    });
    async function requestMountingCheck(action) {
        const response = await fetch('/aatservo', {
            method: 'POST',
            body: new URLSearchParams({ mount_check: action })
        });
        if (!response.ok)
            throw new Error(await response.text() || `Mounting check failed (${response.status})`);
        return response.json();
    }

    mountCheckStartButton.addEventListener('click', async () => {
        if (!isNeutralCompassCheckReady(latestServoData)) {
            mountCheckStatus.textContent = 'Mounting response: blocked; require fresh compass, matching config, clear LOCK, and neutral output';
            return;
        }
        mountRequestPending = true;
        renderCompass(latestServoData);
        try {
            const data = latestServoData.pulse_calibration_supported
                ? await requestMountingCheck('capture') : latestServoData;
            mountCheckStartHeading = data.mounting_start_deg ?? data.calibrated_heading_deg;
            mountCheckPassed = false;
            mountReverseAvailable = false;
            mountCheckStatus.textContent = `Starting heading ${mountCheckStartHeading} deg. Rotate the marked pan clockwise by 90 degrees, then validate.`;
        } catch (error) {
            mountCheckStatus.textContent = `Could not capture mounting response: ${error.message}`;
        } finally {
            mountRequestPending = false;
            renderCompass(latestServoData);
        }
    });
    mountCheckValidateButton.addEventListener('click', async () => {
        if (mountCheckStartHeading === null) {
            mountCheckStatus.textContent = 'Mounting response: capture a starting heading first';
            return;
        }
        mountRequestPending = true;
        mountReverseAvailable = false;
        renderCompass(latestServoData);
        try {
            const response = await fetch('/aatservo', { cache: 'no-store' });
            if (!response.ok)
                throw new Error('Fresh setup status unavailable');
            const data = await response.json();
            renderCompass(data);
            if (!isNeutralCompassCheckReady(data)) {
                mountCheckPassed = false;
                mountCheckStatus.textContent = 'Mounting response: blocked; require fresh compass, matching config, clear LOCK, released GPIO32, and neutral output';
                return;
            }
            let headingChange = data.calibrated_heading_deg - mountCheckStartHeading;
            if (headingChange > 180)
                headingChange -= 360;
            else if (headingChange < -180)
                headingChange += 360;
            if (headingChange >= 70 && headingChange <= 110) {
                if (data.mounting_verification_pending)
                    renderCompass(await requestMountingCheck('verify'));
                mountCheckPassed = true;
                mountCheckStartHeading = null;
                mountCheckStatus.textContent = `Mounting response: PASS, heading increased ${headingChange} deg`;
            } else {
                mountCheckPassed = false;
                mountReverseAvailable = data.pulse_calibration_supported && !data.mounting_reversed
                    && headingChange >= -110 && headingChange <= -70;
                mountCheckStatus.textContent = mountReverseAvailable
                    ? `Heading decreased ${-headingChange} deg. Apply the reversed compass direction, then repeat the hand check.`
                    : `Mounting response: FAIL, heading changed ${headingChange} deg; expected +70 to +110 deg`;
            }
        } catch (error) {
            mountCheckPassed = false;
            mountCheckStatus.textContent = `Mounting response: could not read fresh compass status (${error.message})`;
        } finally {
            mountRequestPending = false;
            renderCompass(latestServoData);
        }
    });
    mountReverseButton.addEventListener('click', async () => {
        if (mountReverseButton.disabled)
            return;
        mountRequestPending = true;
        renderCompass(latestServoData);
        try {
            const data = await requestMountingCheck('reverse');
            mountCheckStartHeading = null;
            mountCheckPassed = false;
            mountReverseAvailable = false;
            renderCompass(data);
            mountCheckStatus.textContent = 'Correction saved. Capture a new starting heading and repeat the clockwise turn before aligning true north.';
        } catch (error) {
            mountCheckStatus.textContent = `Could not apply mounting correction: ${error.message}`;
        } finally {
            mountRequestPending = false;
            renderCompass(latestServoData);
        }
    });
    mountCheckResetButton.addEventListener('click', () => {
        mountCheckStartHeading = null;
        mountCheckPassed = false;
        mountReverseAvailable = false;
        mountCheckStatus.textContent = 'Mounting response: required';
        if (latestServoData)
            renderCompass(latestServoData);
    });

    async function sendSweepCommand(action) {
        if (action === 'owner' && latestServoData && latestServoData.sweep_state === 'complete'
            && latestServoData.sweep_samples > 0 && !sweepExported
            && !confirm('Starting another sweep replaces the current raw data. Start anyway?'))
            return;
        try {
            const response = await fetch('/aatservo', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: new URLSearchParams({ sweep: action })
            });
            if (!response.ok)
                throw new Error(await response.text());
            if (action === 'owner')
                sweepExported = false;
            renderCompass(await response.json());
        } catch (error) {
            sweepStatus.textContent = `Sweep: ${error.message}`;
            if (action === 'stop')
                pulseCalibrationStatus.textContent =
                    `Stop not confirmed: ${error.message}. Disconnect servo power if motion continues.`;
        }
    }

    async function saveCompassCalibration() {
        try {
            const response = await fetch('/aatservo', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: new URLSearchParams({ save_calibration: 1 })
            });
            if (!response.ok)
                throw new Error(await response.text());
            mountCheckStartHeading = null;
            mountCheckPassed = false;
            mountCheckStatus.textContent = 'Mounting response: required after saving a new fit';
            renderCompass(await response.json());
            fitStatus.textContent = 'Compass fit: saved';
        } catch (error) {
            fitStatus.textContent = `Compass fit: ${error.message}`;
        }
    }

    async function downloadSweepCsv() {
        const totalSamples = latestServoData && latestServoData.sweep_samples;
        if (!latestServoData || latestServoData.sweep_state !== 'complete'
            || !totalSamples || sweepExportInProgress)
            return;

        sweepExportInProgress = true;
        sweepDownloadButton.disabled = true;
        let nextStart = 0;
        const rows = [];
        try {
            while (nextStart < totalSamples) {
                const count = Math.min(32, totalSamples - nextStart);
                const query = new URLSearchParams({ start: nextStart, count });
                const response = await fetch(`/aatsweep?${query}`, { cache: 'no-store' });
                if (!response.ok)
                    throw new Error(await response.text());
                const chunk = await response.json();
                if (chunk.schema !== 'aat-sweep-v1' || chunk.sample_count !== totalSamples
                    || chunk.start_index !== nextStart || !Array.isArray(chunk.samples)
                    || chunk.samples.length === 0 || chunk.next_index <= nextStart)
                    throw new Error('Sweep export returned an incomplete or inconsistent chunk');

                rows.push(...chunk.samples);
                nextStart = chunk.next_index;
                sweepExportStatus.textContent = `Downloading raw sweep: ${nextStart}/${totalSamples} samples`;
            }

            const csv = ['sample_index,elapsed_ms,raw_x,raw_y,raw_z',
                ...rows.map(sample => sample.join(','))].join('\r\n');
            const blobUrl = URL.createObjectURL(new Blob([csv], { type: 'text/csv;charset=utf-8' }));
            const downloadLink = document.createElement('a');
            downloadLink.href = blobUrl;
            downloadLink.download = `aat-compass-sweep-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
            document.body.appendChild(downloadLink);
            downloadLink.click();
            downloadLink.remove();
            setTimeout(() => URL.revokeObjectURL(blobUrl), 0);
            sweepExported = true;
            sweepExportStatus.textContent = `Downloaded ${rows.length} raw XYZ sweep samples.`;
        } catch (error) {
            sweepExportStatus.textContent = `Sweep export failed: ${error.message}`;
        } finally {
            sweepExportInProgress = false;
            if (latestServoData)
                renderCompass(latestServoData);
        }
    }

    async function saveTrueNorthReference() {
        try {
            const response = await fetch('/aatservo', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: new URLSearchParams({ north_bearing_deg: 0 })
            });
            if (!response.ok)
                throw new Error(await response.text());
            renderCompass(await response.json());
        } catch (error) {
            northAlignStatus.textContent = `True-north alignment: ${error.message}`;
        }
    }

    const neutralInput = _('pan_neutral_us');
    neutralInput.addEventListener('change', () => {
        const neutralUs = Number(neutralInput.value);
        if (neutralUs < 1450 || neutralUs > 1550)
            return;
        aatServoTestNeutralUs = neutralUs;
        if (panel.style.display !== 'none') {
            renderPulse(neutralUs);
            sendPulse(neutralUs);
        }
    });

    refreshSetupPanel();
}

function initAat() {
    let aatsubmit = _('aatsubmit');
    if (!aatsubmit)
        return;

    aatsubmit.addEventListener('click', callback('Update AAT Parameters', 'An error occurred changing values', '/aatconfig',
        () => {
            const params = new URLSearchParams(new FormData(_('aatconfig')));
            if (Number(_('servomode').value) === 3) {
                params.delete('bear');
                params.delete('elev');
            }
            return params;
        }
    ));
    _('azim_center').addEventListener('change', aatAzimCenterChanged);
    for (const id of ['pan_neutral_us', 'pan_left_max_offset_us', 'pan_right_max_offset_us'])
        _(id).addEventListener('input', updatePanPulseLimits);
    document.querySelectorAll('.aatlive').forEach(
        el => el.addEventListener('change', aatLineElementChanged)
    );

    const servoMode = _('servomode');
    servoMode.addEventListener('change', () => {
        updateServoModeOptions();
        updateContinuousPanSetup(latestConfigData);
        if (Number(servoMode.value) === 3)
            refreshPanRuntime();
    });
    _('continuous-pan-enter-setup').addEventListener('click', async () => {
        if (!window.confirm('Tracking will stop and the pan will enter calibration setup. Continue?'))
            return;
        try {
            const response = await fetch('/aatservo', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: new URLSearchParams({ enter_setup: 1 })
            });
            if (!response.ok)
                throw new Error(await response.text());
            await refreshPanRuntime();
        } catch (error) {
            _('continuous-pan-summary').textContent = `Cannot enter setup: ${error.message}`;
        }
    });
    window.setInterval(() => {
        if (Number(servoMode.value) === 3)
            refreshPanRuntime();
    }, 1000);
}

function initMavLink() {
    // Fetch initial MavLink configuration
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            const data = JSON.parse(this.responseText);
            updateMavLinkConfig(data);
            
            // Start periodic updates of MavLink stats
            updateMavLinkStats();
            setInterval(updateMavLinkStats, 1000);

            const resetButton = _('mavlink_reset_defaults');
            if (resetButton) {
                resetButton.addEventListener('click', resetMavLinkDefaults);
            }
        }
    };
    xmlhttp.open('GET', '/mavlink', true);
    xmlhttp.send();
}
function initOptions() {
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        const data = JSON.parse(this.responseText);
        updateConfig(data);
        setTimeout(get_networks, 2000);
      }
    };
    xmlhttp.open('GET', '/config', true);
    xmlhttp.send();
}

function updateMavLinkConfig(data) {
    _('mavlink_listen_port').value = data.ports.listen;
    _('mavlink_send_port').value = data.ports.send;
}

function updateMavLinkStats() {
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            const data = JSON.parse(this.responseText);
            
            // Update Stats
            _('mavlink_gcs_ip').textContent = data.ip.gcs;
            _('mavlink_packets_down').textContent = data.counters.packets_down;
            _('mavlink_packets_up').textContent = data.counters.packets_up;
            _('mavlink_drops_down').textContent = data.counters.drops_down;
            _('mavlink_overflows_down').textContent = data.counters.overflows_down;
        }
    };
    xmlhttp.open('GET', '/mavlink', true);
    xmlhttp.send();
}

function resetMavLinkDefaults() {
    const defaultSendPort = 14550;
    const defaultListenPort = 14555;

    // Update the input fields
    _('mavlink_listen_port').value = defaultListenPort;
    _('mavlink_send_port').value = defaultSendPort;

    // Send the new values to the server
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (this.readyState == 4) {
            if (this.status == 200) {
                cuteAlert({
                    type: "success",
                    title: "Default Settings Applied",
                    message: "MavLink ports have been reset to default values."
                });
                // Refresh the MavLink stats to reflect the changes
                updateMavLinkStats();
            } else {
                cuteAlert({
                    type: "error",
                    title: "Error",
                    message: "Failed to apply default settings. Please try again."
                });
            }
        }
    };
    xmlhttp.open('POST', '/setmavlink', true);
    xmlhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
    xmlhttp.send(`listen=${defaultListenPort}&send=${defaultSendPort}`);
}

function updateConfig(data) {
    latestConfigData = data;
    let config = data.config;
    if (config.mode==="STA") {
        _('stamode').style.display = 'block';
        if (!config['aat']) {
            if (_('rtctab')) _('rtctab').style.display = 'table-cell';
        }
        _('ssid').textContent = config.ssid;
    } else {
        _('apmode').style.display = 'block';
        if (config.ssid) {
            _('homenet').textContent = config.ssid;
        } else {
            _('connect').style.display = 'none';
        }
    }
    if (config['head-tracking']) {
        if (_('httab')) _('httab').style.display = 'table-cell';
    }
    if((!data.stm32 || data.stm32==="no") && _('tx_tab')) {
        mui.tabs.activate('pane-justified-2');
        _('tx_tab').style.display = 'none';
    }
    if(config['product_name'] && _('product_name')) _('product_name').textContent = config['product_name'];

    updateAatConfig(config);
    updateContinuousPanSetup(data);
}

function updateAatConfig(config)
{
    if (!config.hasOwnProperty('aat'))
        return;
    _('aattab').style.display = 'table-cell';

    // AAT
    _('servosmoo').value = config.aat.servosmoo;
    _('servomode').value = config.aat.servomode;
    updateServoModeOptions();
    _('azim_center').value = config.aat.azim_center;
    _('azim_min').value = config.aat.azim_min;
    _('azim_max').value = config.aat.azim_max;
    _('pan_neutral_us').value = config.aat.pan_neutral_us;
    _('pan_deadband_degrees').value = config.aat.pan_deadband_degrees;
    _('pan_left_max_offset_us').value = config.aat.pan_left_max_offset_us;
    _('pan_right_max_offset_us').value = config.aat.pan_right_max_offset_us;
    updatePanPulseLimits();
    aatServoTestNeutralUs = config.aat.pan_neutral_us;
    _('azim_sff').checked = config.aat.azim_sff === 1;
    _('elev_min').value = config.aat.elev_min;
    _('elev_max').value = config.aat.elev_max;
    _('satmin').value = config.aat.satmin;
    aatAzimCenterChanged();

    // VBAT
    _('vbat_offset').value = config.vbat.offset;
    _('vbat_scale').value = config.vbat.scale;
}

function updatePanPulseLimits()
{
    const neutralInput = _('pan_neutral_us');
    const leftInput = _('pan_left_max_offset_us');
    const rightInput = _('pan_right_max_offset_us');
    const preview = _('pan_pulse_limits');
    const neutral = neutralInput.valueAsNumber;
    if (!Number.isInteger(neutral) || !neutralInput.validity.valid) {
        leftInput.max = rightInput.max = 550;
        preview.textContent = 'Enter a neutral pulse from 1450 to 1550 us to calculate the limits.';
        return;
    }

    leftInput.max = neutral - 1000;
    rightInput.max = 2000 - neutral;
    if (!Number.isInteger(leftInput.valueAsNumber) || !leftInput.validity.valid
        || !Number.isInteger(rightInput.valueAsNumber) || !rightInput.validity.valid) {
        preview.textContent = `Enter a left offset of 5-${leftInput.max} us and a right offset of 5-${rightInput.max} us.`;
        return;
    }
    preview.textContent = `Pulse limits: left ${neutral - leftInput.valueAsNumber} us, right ${neutral + rightInput.valueAsNumber} us.`;
}

function updateServoModeOptions()
{
    const positionalMode = Number(_('servomode').value) !== 3;
    _('positional-azimuth-options').style.display = positionalMode ? '' : 'none';
    _('positional-elevation-test').style.display = positionalMode ? '' : 'none';
    _('positional-servo-speed').style.display = positionalMode ? '' : 'none';
    _('continuous-servo-speed').style.display = positionalMode ? 'none' : '';
}

function updateContinuousPanSetup(data)
{
    const panel = _('continuous-pan-setup');
    const panelContainer = _('continuous-pan-panel');
    const selected = Number(_('servomode').value) === 3;
    if (!panel)
        return;

    panelContainer.style.display = selected ? '' : 'none';
    panel.style.display = selected ? 'block' : 'none';
    if (!selected)
        return;

    const summary = _('continuous-pan-summary');
    const setupButton = _('continuous-pan-enter-setup');
    const fitStatus = _('continuous-pan-fit');
    const northStatus = _('continuous-pan-north');
    const compassStatus = _('continuous-pan-compass');
    const gpsStatus = _('continuous-pan-gps');
    const referenceStatus = _('continuous-pan-reference');
    const outputStatus = _('continuous-pan-output');
    const savedConfig = data && data.config && data.config.aat;
    const runtime = data && data.pan_runtime;

    if (!data || !savedConfig) {
        summary.textContent = 'Waiting for AAT configuration.';
        return;
    }

    fitStatus.textContent = savedConfig.pan_compass_valid ? 'Compass fit: saved' : 'Compass fit: required';
    northStatus.textContent = savedConfig.pan_north_aligned
        ? 'True-north calibration: saved'
        : 'True-north calibration: required';

    if (!runtime) {
        setupButton.style.display = 'none';
        summary.textContent = 'Continuous pan is unavailable in this firmware. Use the CompassPan firmware to run mode 3.';
        compassStatus.textContent = 'Compass runtime: unavailable';
        gpsStatus.textContent = 'GPS runtime: unavailable';
        referenceStatus.textContent = 'Reference: unavailable';
        outputStatus.textContent = 'Output: unavailable';
        return;
    }

    setupButton.style.display = Number(savedConfig.servomode) === 3
        && runtime.mode_active && !runtime.setup_active ? 'inline-block' : 'none';

    if (runtime.setup_active) {
        if (refreshAatServoSetupPanel)
            refreshAatServoSetupPanel();
        summary.textContent = !savedConfig.pan_compass_valid
            ? 'Setup is active. Start with the compass fit.'
            : !savedConfig.pan_north_aligned
                ? 'Compass fit saved. Check the mounting response, then save true north.'
                : !savedConfig.pan_pulse_calibration_valid
                    ? 'Compass and true north saved. Confirm mechanical home before running pulse calibration.'
                    : 'Pulse calibration saved. Tracking remains inhibited while setup is active.';
        compassStatus.textContent = runtime.compass_detected
            ? (savedConfig.pan_compass_valid ? 'Compass: detected; fit saved' : 'Compass: detected; fit setup pending')
            : 'Compass: searching';
        gpsStatus.textContent = 'Aircraft GPS: checked when tracking starts';
        referenceStatus.textContent = 'Setup: tracking inhibited';
        outputStatus.textContent = 'Tracking output inhibited; use guarded setup controls below.';
        return;
    }

    if (!runtime.mode_active) {
        compassStatus.textContent = 'Compass runtime: checked after mode 3 starts';
        gpsStatus.textContent = 'Aircraft GPS: checked after mode 3 starts';
        referenceStatus.textContent = 'CompassPan reference: inactive';
        outputStatus.textContent = 'CompassPan output: inactive';
        summary.textContent = Number(savedConfig.servomode) === 3
            ? 'Mode 3 is saved but inactive. Reboot the tracker to start continuous pan.'
            : 'Mode 3 is selected here but not saved. Save settings, then reboot to apply it.';
        return;
    }

    const compassSampleValid = runtime.compass_sample_valid !== undefined
        ? runtime.compass_sample_valid : runtime.compass_fresh;
    const compassReady = runtime.compass_detected
        && compassSampleValid
        && runtime.compass_config_matches_requested
        && runtime.trigger_status_valid
        && runtime.trigger_lock_clear;
    compassStatus.textContent = !runtime.compass_detected ? 'Compass: not detected'
        : !runtime.compass_config_matches_requested ? 'Compass: configuration mismatch'
        : !runtime.trigger_status_valid ? 'Compass: trigger status unavailable'
        : !runtime.trigger_lock_clear ? 'Compass: waiting for LOCK to clear'
        : !compassSampleValid ? 'Compass: waiting for a fresh valid sample'
        : 'Compass: ready';
    gpsStatus.textContent = runtime.gps_fresh ? 'Aircraft GPS: fresh' : 'Aircraft GPS: stale or unavailable';
    referenceStatus.textContent = ['Waiting for button release', 'Not referenced', 'Referenced', 'Safety fault'][runtime.safety_state]
        || 'Reference: unknown';
    outputStatus.textContent = `Pan ${runtime.current_pan_deg} deg; target ${runtime.target_pan_deg} deg; error ${runtime.error_deg} deg; pulse ${runtime.applied_pulse_us} us`;

    if (Number(savedConfig.servomode) !== 3)
        summary.textContent = 'Mode 3 is selected here but not saved. Save settings, then reboot to apply it.';
    else if (!runtime.mode_active)
        summary.textContent = 'Mode 3 is saved but inactive. Reboot the tracker to start continuous pan.';
    else if (runtime.block_reason && runtime.block_reason !== 'none') {
        const blockMessages = {
            'setup-active': 'Tracking inhibited: setup mode is active.',
            'startup-alignment-required': 'Tracking inhibited: confirm the mechanical home at startup.',
            'compass-invalid': 'Tracking inhibited: compass feedback is stale or invalid.',
            'reference-required': 'Tracking inhibited: pan is not referenced.',
            'gps-stale': 'Tracking inhibited: aircraft GPS is stale or unavailable.',
            'safety-fault': 'Tracking inhibited: a pan safety fault requires a new reference.'
        };
        summary.textContent = blockMessages[runtime.block_reason]
            || `Tracking inhibited: ${runtime.block_reason}.`;
    }
    else if (!compassSampleValid)
        summary.textContent = 'Tracking inhibited: compass sample, configuration, or trigger status is not valid.';
    else if (!runtime.startup_alignment_complete)
        summary.textContent = savedConfig.pan_north_aligned
            ? 'Starting: place the pan at its marked mechanical home and press GPIO32.'
            : 'Starting: point the pan true north and press GPIO32 to save north and set home.';
    else if (!savedConfig.pan_compass_valid)
        summary.textContent = 'Tracking inhibited: save a valid compass axis fit first.';
    else if (!runtime.north_aligned)
        summary.textContent = 'Tracking inhibited: true-north calibration is required.';
    else if (runtime.safety_state !== 2)
        summary.textContent = 'Tracking inhibited: pan is not referenced or a safety fault is active.';
    else if (!compassReady)
        summary.textContent = 'Tracking inhibited: compass checks are not ready.';
    else if (!runtime.gps_fresh)
        summary.textContent = 'Tracking inhibited: waiting for fresh aircraft GPS.';
    else
        summary.textContent = 'Continuous pan is tracking with valid compass, reference, and aircraft GPS.';
}

async function refreshPanRuntime()
{
    try {
        const response = await fetch('/config', { cache: 'no-store' });
        if (!response.ok)
            throw new Error('Configuration unavailable');
        const data = await response.json();
        latestConfigData = data;
        updateContinuousPanSetup(data);
    } catch (_error) {
        const summary = _('continuous-pan-summary');
        if (summary)
            summary.textContent = 'Cannot read continuous-pan status.';
    }
}

function get_networks() {
    var json_url = 'networks.json';
    xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            var data = JSON.parse(this.responseText);
            _('loader').style.display = 'none';
            autocomplete(_('network'), data);
        }
    };
    xmlhttp.open("POST", json_url, true);
    xmlhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
    xmlhttp.send();
}

function hasErrorParameter() {
    var tmp = [], result = false;
    location.search
        .substr(1)
        .split("&")
        .forEach(function (item) {
            tmp = item.split("=");
            if (tmp[0] === "error") result = true;
        });
    return result;
}

function show(elements, specifiedDisplay) {
    elements = elements.length ? elements : [elements];
    for (var index = 0; index < elements.length; index++) {
        elements[index].style.display = specifiedDisplay || 'block';
    }
}

var elements = document.querySelectorAll('#failed');
if (hasErrorParameter()) show(elements);

function autocomplete(inp, arr) {
    /*the autocomplete function takes two arguments,
    the text field element and an array of possible autocompleted values:*/
    var currentFocus;

    /*execute a function when someone writes in the text field:*/
    function handler(e) {
        var a, b, i, val = this.value;
        /*close any already open lists of autocompleted values*/
        closeAllLists();
        currentFocus = -1;
        /*create a DIV element that will contain the items (values):*/
        a = document.createElement("DIV");
        a.setAttribute("id", this.id + "autocomplete-list");
        a.setAttribute("class", "autocomplete-items");
        /*append the DIV element as a child of the autocomplete container:*/
        this.parentNode.appendChild(a);
        /*for each item in the array...*/
        for (i = 0; i < arr.length; i++) {
            /*check if the item starts with the same letters as the text field value:*/
            if (arr[i].substr(0, val.length).toUpperCase() == val.toUpperCase()) {
                /*create a DIV element for each matching element:*/
                b = document.createElement("DIV");
                /*make the matching letters bold:*/
                b.innerHTML = "<strong>" + arr[i].substr(0, val.length) + "</strong>";
                b.innerHTML += arr[i].substr(val.length);
                /*insert a input field that will hold the current array item's value:*/
                b.innerHTML += "<input type='hidden' value='" + arr[i] + "'>";
                /*execute a function when someone clicks on the item value (DIV element):*/
                b.addEventListener("click", ((arg) => (e) => {
                    /*insert the value for the autocomplete text field:*/
                    inp.value = arg.getElementsByTagName("input")[0].value;
                    /*close the list of autocompleted values,
                    (or any other open lists of autocompleted values:*/
                    closeAllLists();
                })(b));
                a.appendChild(b);
            }
        }
    }
    inp.addEventListener("input", handler);
    inp.addEventListener("click", handler);

    /*execute a function presses a key on the keyboard:*/
    inp.addEventListener("keydown", (e) => {
        var x = _(this.id + "autocomplete-list");
        if (x) x = x.getElementsByTagName("div");
        if (e.keyCode == 40) {
            /*If the arrow DOWN key is pressed,
            increase the currentFocus variable:*/
            currentFocus++;
            /*and and make the current item more visible:*/
            addActive(x);
        } else if (e.keyCode == 38) { //up
            /*If the arrow UP key is pressed,
            decrease the currentFocus variable:*/
            currentFocus--;
            /*and and make the current item more visible:*/
            addActive(x);
        } else if (e.keyCode == 13) {
            /*If the ENTER key is pressed, prevent the form from being submitted,*/
            e.preventDefault();
            if (currentFocus > -1) {
                /*and simulate a click on the "active" item:*/
                if (x) x[currentFocus].click();
            }
        }
    });
    function addActive(x) {
        /*a function to classify an item as "active":*/
        if (!x) return false;
        /*start by removing the "active" class on all items:*/
        removeActive(x);
        if (currentFocus >= x.length) currentFocus = 0;
        if (currentFocus < 0) currentFocus = (x.length - 1);
        /*add class "autocomplete-active":*/
        x[currentFocus].classList.add("autocomplete-active");
    }
    function removeActive(x) {
        /*a function to remove the "active" class from all autocomplete items:*/
        for (var i = 0; i < x.length; i++) {
            x[i].classList.remove("autocomplete-active");
        }
    }
    function closeAllLists(elmnt) {
        /*close all autocomplete lists in the document,
        except the one passed as an argument:*/
        var x = document.getElementsByClassName("autocomplete-items");
        for (var i = 0; i < x.length; i++) {
            if (elmnt != x[i] && elmnt != inp) {
                x[i].parentNode.removeChild(x[i]);
            }
        }
    }
    /*execute a function when someone clicks in the document:*/
    document.addEventListener("click", (e) => {
        closeAllLists(e.target);
    });
}

//=========================================================

function uploadFile(type_suffix) {
    var file = _("firmware_file_" + type_suffix).files[0];
    var formdata = new FormData();
    formdata.append("type", type_suffix);
    formdata.append("upload", file, file.name);
    var ajax = new XMLHttpRequest();
    ajax.upload.addEventListener("progress", progressHandler(type_suffix), false);
    ajax.addEventListener("load", completeHandler(type_suffix), false);
    ajax.addEventListener("error", errorHandler(type_suffix), false);
    ajax.addEventListener("abort", abortHandler(type_suffix), false);
    ajax.open("POST", "/update");
    ajax.send(formdata);
}

function progressHandler(type_suffix) {
    return function (event) {
        //_("loaded_n_total").innerHTML = "Uploaded " + event.loaded + " bytes of " + event.total;
        var percent = Math.round((event.loaded / event.total) * 100);
        _("progressBar_" + type_suffix).value = percent;
        _("status_" + type_suffix).innerHTML = percent + "% uploaded... please wait";
    }
}

function completeHandler(type_suffix) {
    return function(event) {
        _("status_" + type_suffix).innerHTML = "";
        _("progressBar_" + type_suffix).value = 0;
        var data = JSON.parse(event.target.responseText);
        if (data.status === 'ok') {
            function show_message() {
                cuteAlert({
                    type: 'success',
                    title: "Update Succeeded",
                    message: data.msg
                });
            }
            // This is basically a delayed display of the success dialog with a fake progress
            var percent = 0;
            var interval = setInterval(()=>{
                percent = percent + 1;
                _("progressBar_" + type_suffix).value = percent;
                _("status_" + type_suffix).innerHTML = percent + "% flashed... please wait";
                if (percent == 100) {
                    clearInterval(interval);
                    _("status_" + type_suffix).innerHTML = "";
                    _("progressBar_" + type_suffix).value = 0;
                    show_message();
                }
            }, 100);
        } else if (data.status === 'mismatch') {
            cuteAlert({
                type: 'question',
                title: "Targets Mismatch",
                message: data.msg,
                confirmText: "Flash anyway",
                cancelText: "Cancel"
            }).then((e)=>{
                xmlhttp = new XMLHttpRequest();
                xmlhttp.onreadystatechange = function () {
                    if (this.readyState == 4) {
                        _("status_" + type_suffix).innerHTML = "";
                        _("progressBar_" + type_suffix).value = 0;
                        if (this.status == 200) {
                            var data = JSON.parse(this.responseText);
                            cuteAlert({
                                type: "info",
                                title: "Force Update",
                                message: data.msg
                            });
                        }
                        else {
                            cuteAlert({
                                type: "error",
                                title: "Force Update",
                                message: "An error occurred trying to force the update"
                            });
                        }
                    }
                };
                xmlhttp.open("POST", "/forceupdate", true);
                var data = new FormData();
                data.append("action", e);
                xmlhttp.send(data);
            });
        } else {
            cuteAlert({
                type: 'error',
                title: "Update Failed",
                message: data.msg
            });
        }
    }
}

function errorHandler(type_suffix) {
    return function(event) {
        _("status_" + type_suffix).innerHTML = "";
        _("progressBar_" + type_suffix).value = 0;
        cuteAlert({
            type: "error",
            title: "Update Failed",
            message: event.target.responseText
        });
    }
}

function abortHandler(type_suffix) {
    return function(event) {
        _("status_" + type_suffix).innerHTML = "";
        _("progressBar_" + type_suffix).value = 0;
        cuteAlert({
            type: "info",
            title: "Update Aborted",
            message: event.target.responseText
        });
    }
}

if (_('upload_form_tx')) {
    _('upload_form_tx').addEventListener('submit', (e) => {
        e.preventDefault();
        uploadFile("tx");
    });
}

if(_('upload_form_bp')) {
    _('upload_form_bp').addEventListener('submit', (e) => {
        e.preventDefault();
        uploadFile("bp");
    });
}

//=========================================================

function callback(title, msg, url, getdata) {
    return function(e) {
        e.stopPropagation();
        e.preventDefault();
        xmlhttp = new XMLHttpRequest();
        xmlhttp.onreadystatechange = function () {
            if (this.readyState == 4) {
                if (this.status == 200) {
                    cuteAlert({
                        type: "info",
                        title: title,
                        message: this.responseText
                    });
                }
                else {
                    cuteAlert({
                        type: "error",
                        title: title,
                        message: msg
                    });
                }
            }
        };
        xmlhttp.open("POST", url, true);
        if (getdata) data = getdata();
        else data = null;
        xmlhttp.send(data);
    }
}

function aatAzimCenterChanged()
{
    // Update the slider labels to represent the new orientation
    let labels;
    switch (parseInt(_('azim_center').selectedIndex))
    {
        default: /* fallthrough */
        case 0: labels = 'SWNES'; break; // N
        case 1: labels = 'WNESW'; break; // E
        case 2: labels = 'NESWN'; break; // S
        case 3: labels = 'ESWNE'; break; // W
    }
    let markers = _('bear_markers');
    for (i=0; i<markers.options.length; ++i)
        markers.options[i].label = labels[i];
}

function aatLineElementChanged()
{
    fetch("/aatconfig", {
        method: "POST",
        headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: new URLSearchParams({
            'bear': _('bear').value,
            'elev': _('elev').value,
            'azim_sff': _('azim_sff').checked ? 1 : 0,
        })
    });
  }

_('sethome').addEventListener('submit', callback("Set Home Network", "An error occurred setting the home network", "/sethome", function() {
    return new FormData(_('sethome'));
}));
_('connect').addEventListener('click', callback("Connect to Home Network", "An error occurred connecting to the Home network", "/connect", null));
_('access').addEventListener('click', callback("Access Point", "An error occurred starting the Access Point", "/access", null));
_('forget').addEventListener('click', callback("Forget Home Network", "An error occurred forgetting the home network", "/forget", null));
if (_('setrtc')) _('setrtc').addEventListener('submit', callback("Set RTC Time", "An error occurred setting the RTC time", "/setrtc", function() {
    return new FormData(_('setrtc'));
}));

//=========================================================

// Alert box design by Igor Ferrão de Souza: https://www.linkedin.com/in/igor-ferr%C3%A3o-de-souza-4122407b/

function cuteAlert({
    type,
    title,
    message,
    buttonText = "OK",
    confirmText = "OK",
    cancelText = "Cancel",
    closeStyle,
  }) {
    return new Promise((resolve) => {
      setInterval(() => {}, 5000);
      const body = document.querySelector("body");

      const scripts = document.getElementsByTagName("script");

      let closeStyleTemplate = "alert-close";
      if (closeStyle === "circle") {
        closeStyleTemplate = "alert-close-circle";
      }

      let btnTemplate = `<div><button class="alert-button mui-btn mui-btn--primary">${buttonText}</button></div>`;
      if (type === "question") {
        btnTemplate = `
<div class="question-buttons">
  <button class="confirm-button mui-btn mui-btn--danger">${confirmText}</button>
  <button class="cancel-button mui-btn">${cancelText}</button>
</div>
`;
      }

      let svgTemplate = `
<svg class="alert-img" xmlns="http://www.w3.org/2000/svg" fill="#fff" viewBox="0 0 52 52" xmlns:v="https://vecta.io/nano">
<path d="M26 0C11.664 0 0 11.663 0 26s11.664 26 26 26 26-11.663 26-26S40.336 0 26 0zm0 50C12.767 50 2 39.233 2 26S12.767 2 26 2s24 10.767 24 24-10.767 24-24
24zm9.707-33.707a1 1 0 0 0-1.414 0L26 24.586l-8.293-8.293a1 1 0 0 0-1.414 1.414L24.586 26l-8.293 8.293a1 1 0 0 0 0 1.414c.195.195.451.293.707.293s.512-.098.707
-.293L26 27.414l8.293 8.293c.195.195.451.293.707.293s.512-.098.707-.293a1 1 0 0 0 0-1.414L27.414 26l8.293-8.293a1 1 0 0 0 0-1.414z"/>
</svg>
`;
      if (type === "success") {
        svgTemplate = `
<svg class="alert-img" xmlns="http://www.w3.org/2000/svg" fill="#fff" viewBox="0 0 52 52" xmlns:v="https://vecta.io/nano">
<path d="M26 0C11.664 0 0 11.663 0 26s11.664 26 26 26 26-11.663 26-26S40.336 0 26 0zm0 50C12.767 50 2 39.233 2 26S12.767 2 26 2s24 10.767 24 24-10.767 24-24
24zm12.252-34.664l-15.369 17.29-9.259-7.407a1 1 0 0 0-1.249 1.562l10 8a1 1 0 0 0 1.373-.117l16-18a1 1 0 1 0-1.496-1.328z"/>
</svg>
`;
      }
      if (type === "info") {
        svgTemplate = `
<svg class="alert-img" xmlns="http://www.w3.org/2000/svg" fill="#fff" viewBox="0 0 64 64" xmlns:v="https://vecta.io/nano">
<path d="M38.535 47.606h-4.08V28.447a1 1 0 0 0-1-1h-4.52a1 1 0 1 0 0 2h3.52v18.159h-5.122a1 1 0 1 0 0 2h11.202a1 1 0 1 0 0-2z"/>
<circle cx="32" cy="18" r="3"/><path d="M32 0C14.327 0 0 14.327 0 32s14.327 32 32 32 32-14.327 32-32S49.673 0 32 0zm0 62C15.458 62 2 48.542 2 32S15.458 2 32 2s30 13.458 30 30-13.458 30-30 30z"/>
</svg>
`;
      }

      const template = `
<div class="alert-wrapper">
  <div class="alert-frame">
    <div class="alert-header ${type}-bg">
      <span class="${closeStyleTemplate}">X</span>
      ${svgTemplate}
    </div>
    <div class="alert-body">
      <span class="alert-title">${title}</span>
      <span class="alert-message">${message}</span>
      ${btnTemplate}
    </div>
  </div>
</div>
`;

      body.insertAdjacentHTML("afterend", template);

      const alertWrapper = document.querySelector(".alert-wrapper");
      const alertFrame = document.querySelector(".alert-frame");
      const alertClose = document.querySelector(`.${closeStyleTemplate}`);

      function resolveIt() {
        alertWrapper.remove();
        resolve();
      }
      function confirmIt() {
        alertWrapper.remove();
        resolve("confirm");
      }
      function stopProp(e) {
        e.stopPropagation();
      }

      if (type === "question") {
        const confirmButton = document.querySelector(".confirm-button");
        const cancelButton = document.querySelector(".cancel-button");

        confirmButton.addEventListener("click", confirmIt);
        cancelButton.addEventListener("click", resolveIt);
      } else {
        const alertButton = document.querySelector(".alert-button");

        alertButton.addEventListener("click", resolveIt);
      }

      alertClose.addEventListener("click", resolveIt);
      alertWrapper.addEventListener("click", resolveIt);
      alertFrame.addEventListener("click", stopProp);
    });
  }

//=========================================================

if (_('httab')) _('httab').addEventListener('mui.tabs.showstart', start);

var websock;
var Euler = {heading: 0.0, pitch: 0.0, roll: 0.0};

const loadScript = (FILE_URL, async = true, type = "text/javascript") => {
    return new Promise((resolve, reject) => {
        try {
            const scriptEle = document.createElement("script");
            scriptEle.type = type;
            scriptEle.async = async;
            scriptEle.src = FILE_URL;

            scriptEle.addEventListener("load", (ev) => {
                resolve({ status: true });
            });

            scriptEle.addEventListener("error", (ev) => {
                reject({
                    status: false,
                    message: `Failed to load the script ${FILE_URL}`
                });
            });

            document.body.appendChild(scriptEle);
        } catch (error) {
            reject(error);
        }
    });
};

let ht_loaded = false;
function start() {
    if (!ht_loaded) {
        loadScript('p5.js').then(()=>{
            ht_loaded = true;
            websock = new WebSocket('ws://' + window.location.hostname + '/ws');
            // websock.onopen = function(evt) {
            //     console.log('websock open');
            //     var e = document.getElementById('webSockStatus');
            //     e.style.backgroundColor = 'green';
            // };
            // websock.onclose = function(evt) {
            //     console.log('websock close');
            //     var e = document.getElementById('webSockStatus');
            //     e.style.backgroundColor = 'red';
            // };
            websock.onerror = function(evt) { console.log(evt); };
            websock.onmessage = async function(evt) {
                d = JSON.parse(evt.data);
                if (d['done']) {
                    calibrationOff();
                    await cuteAlert({
                        type: 'info',
                        title: "Calibration",
                        message: "Calibration successful",
                        confirmText: "OK",
                    });
                }
                if (d['orientation']) {
                    _('x-angle').value = d.pitch;
                    _('y-angle').value = d.roll;
                    _('z-angle').value = d.heading;
                    _('label-x').textContent = d.pitch;
                    _('label-y').textContent = d.roll;
                    _('label-z').textContent = d.heading;
                    if (!d.hasIMU) {
                        show(document.querySelectorAll('.hasIMU'), 'none');
                    }
                }
                if (d['heading']) {
                    Euler = d;
                    _('angle-x').textContent = Euler.roll;
                    _('angle-y').textContent = Euler.pitch;
                    _('angle-z').textContent = Euler.heading;
                }
            };
        })
    }
}

let plane;
let tex;

function preload() {
    plane = loadModel('airplane.obj', true);
    tex = loadImage('texture.gif');
}

function setup() {
    var canvas = createCanvas(500, 500, WEBGL);
    canvas.parent('canvas-holder');
}

function draw() {
    background(192);

    rotateY(radians(Euler.heading + 180)); // Add 180 degrees so the plane is facing away at zero
    rotateZ(radians(Euler.pitch));
    rotateX(radians(-Euler.roll)); // Invert the about the pitch axis (i.e. roll is opposite)

    push();
    stroke('#CCC');
    strokeWeight(0.5);
    for (let x=-width/2; x <= width/2; x +=20) {
        line(x, 0, -height/2, x, 0, height/2);
    }
    for (let z=-height/2; z <= height/2; z +=20) {
        line(-width/2, 0, z, width/2, 0, z);
    }
    pop();

    push();
    noStroke();
    scale(2);
    translate(0,-26,0);
    texture(tex);
    model(plane);
    pop();
}

if (_('set-center')) _('set-center').addEventListener('click', () => {websock.send('sc');});
if (_('cal-gyro')) _('cal-gyro').addEventListener('click', calibrateIMU);
if (_('reset-board')) _('reset-board').addEventListener('click', (e) => {
    _('x-angle').value = 0;
    _('y-angle').value = 0;
    _('z-angle').value = 0;
    setOrientation(e);
    websock.send('ro');
});
if (_('save-orientation')) _('save-orientation').addEventListener('click', saveOrientation);
if (_('x-angle')) _('x-angle').addEventListener('input', setOrientation);
if (_('y-angle')) _('y-angle').addEventListener('input', setOrientation);
if (_('z-angle')) _('z-angle').addEventListener('input', setOrientation);

async function calibrateIMU() {
    await cuteAlert({
        type: 'info',
        title: "Calibrate IMU",
        message: "Place the board flat on the table and wait until the succeeded popup appears",
        confirmText: "Calibrate",
        cancelText: "Cancel"
    }).then((e)=>{
        websock.send('ci');
        calibrationOn();
    });
}

function setOrientation(e) {
    _('label-x').textContent = _('x-angle').value;
    _('label-y').textContent = _('y-angle').value;
    _('label-z').textContent = _('z-angle').value;
    websock.send('o:' + _('x-angle').value + ':' + _('y-angle').value + ':' + _('z-angle').value);
}

function saveOrientation() {
    websock.send('sv');
    cuteAlert({
        type: 'info',
        title: "Save Board Orientation",
        message: "Board orientation has been saved to configuration",
        confirmText: "OK"
    });
}

function calibrationOn() {
    _('main').classList.add('loading');
    let calibrating = document.createElement('div')
    calibrating.innerHTML = '<div class="ring">Calibrating<span></span></div>'
    mui.overlay('on', {
        'keyboard': false,
        'static': true
    }, calibrating);
}

function calibrationOff() {
    _('main').classList.remove('loading');
    mui.overlay('off');
}
