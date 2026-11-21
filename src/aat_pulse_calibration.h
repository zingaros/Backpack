#pragma once

#include "aat_config_validation.h"

namespace AatServo
{
class PulseCalibration
{
public:
    enum class State : uint8_t { Idle, Left, Right, Complete, Fault };
    enum class Fault : uint8_t {
        None, Configuration, SafetyGate, StageTimeout, SettleTimeout,
        WrongDirection, Travel, Timeout, Output
    };
    static constexpr uint32_t STAGE_MS = 3000;
    static constexpr uint32_t HEADING_WINDOW_MS = 500;
    static constexpr uint32_t MAX_SAMPLE_GAP_MS = 200;
    static constexpr uint32_t SETTLE_MIN_MS = 2000;
    static constexpr uint32_t SETTLE_TIMEOUT_MS = 3000;
    static constexpr uint32_t TOTAL_TIMEOUT_MS = 600000;
    static constexpr int32_t NOISE_BAND_MDEG = 6000;
    static constexpr int32_t SETTLE_TREND_MDEG = 1500;
    static constexpr int32_t MIN_PROGRESS_MDEG = 250;
    static constexpr int32_t MAX_STAGE_TRAVEL_DEGREES = 15;

    bool start(uint32_t now, uint16_t neutralUs, uint16_t leftOffsetUs,
        uint16_t rightOffsetUs, uint32_t sampleSequence)
    {
        if (active())
            return false;
        _neutralUs = neutralUs;
        _fault = Fault::None;
        _finishing = false;
        _leftStartUs = _rightStartUs = 0;
        if (!AatConfigValidation::isPanPulseEnvelopeValid(
            neutralUs, leftOffsetUs, rightOffsetUs))
        {
            fail(Fault::Configuration);
            return false;
        }
        _leftLimitUs = neutralUs - leftOffsetUs;
        _rightLimitUs = neutralUs + rightOffsetUs;
        _candidateUs = neutralUs - AatConfigDefaults::PAN_PULSE_CALIBRATION_STEP_US;
        _state = State::Left;
        _startedMs = now;
        settle(now, sampleSequence);
        return true;
    }

    uint16_t update(uint32_t now, bool safetyReady, int32_t panDegrees,
        uint16_t appliedUs, bool sampleValid, uint32_t sequence)
    {
        if (!active())
            return _neutralUs;
        if (!safetyReady || panDegrees <= -90 || panDegrees >= 90)
            fail(Fault::SafetyGate);
        else if (now - _startedMs >= TOTAL_TIMEOUT_MS)
            fail(Fault::Timeout);
        if (!active())
            return _neutralUs;

        if (!_settling && (panDegrees - _stagePanDegrees >= MAX_STAGE_TRAVEL_DEGREES
            || _stagePanDegrees - panDegrees >= MAX_STAGE_TRAVEL_DEGREES))
        {
            fail(Fault::Travel);
            return _neutralUs;
        }
        const uint16_t expectedUs = _settling ? _neutralUs : _candidateUs;
        if (!_outputObserved)
        {
            if (now - _phaseStartedMs >= STAGE_MS)
            {
                fail(Fault::Output);
                return _neutralUs;
            }
            if (appliedUs != expectedUs)
                return requestedPulseUs();
            // Only new heading samples after pulse application may qualify motion.
            _outputObserved = true;
            _lastSequence = sequence;
            _lastSampleAtMs = now;
            return requestedPulseUs();
        }
        if (appliedUs != expectedUs)
        {
            fail(Fault::Output);
            return _neutralUs;
        }

        if (!sampleValid || now - _lastSampleAtMs > MAX_SAMPLE_GAP_MS)
            resetObservation(now);
        if (sequence != _lastSequence)
        {
            _lastSequence = sequence;
            if (sampleValid)
                observeHeading(now, panDegrees * 1000);
        }
        if (!active())
            return _neutralUs;

        const uint32_t elapsedMs = now - _phaseStartedMs;
        if (_settling)
        {
            if (elapsedMs >= SETTLE_MIN_MS && headingSettled())
            {
                const int32_t baseline = (_means[0] + _means[1] + _means[2] + _means[3]) / 4;
                _settling = false;
                _outputObserved = false;
                _samples = 0;
                _phaseStartedMs = now;
                _stagePanDegrees = panDegrees;
                resetObservation(now);
                _motionBaselineMdeg = _previousMeanMdeg = baseline;
                _haveMotionBaseline = true;
            }
            else if (elapsedMs >= SETTLE_TIMEOUT_MS)
                fail(Fault::SettleTimeout);
        }
        else if (_samples >= 3 && directedDisplacement() > NOISE_BAND_MDEG)
        {
            if (_state == State::Left)
            {
                _leftStartUs = _candidateUs;
                _state = State::Right;
                _candidateUs = _neutralUs + AatConfigDefaults::PAN_PULSE_CALIBRATION_STEP_US;
            }
            else
            {
                _rightStartUs = _candidateUs;
                // Completion is published only after the load settles at neutral.
                _finishing = true;
            }
            settle(now, sequence);
        }
        else if (elapsedMs >= STAGE_MS)
        {
            if ((_state == State::Left && _candidateUs == _leftLimitUs)
                || (_state == State::Right && _candidateUs == _rightLimitUs))
                fail(Fault::StageTimeout);
            else
            {
                const uint16_t step = AatConfigDefaults::PAN_PULSE_CALIBRATION_STEP_US;
                if (_state == State::Left)
                    _candidateUs = _candidateUs - _leftLimitUs > step
                        ? _candidateUs - step : _leftLimitUs;
                else
                    _candidateUs = _rightLimitUs - _candidateUs > step
                        ? _candidateUs + step : _rightLimitUs;
                settle(now, sequence);
            }
        }
        if (_finishing && _settling == false && active())
            _state = State::Complete;
        return requestedPulseUs();
    }

    void cancel() { _state = State::Idle; _finishing = false; }
    void fail(Fault fault) { _fault = fault; _state = State::Fault; }
    bool active() const { return _state == State::Left || _state == State::Right; }
    State state() const { return _state; }
    Fault fault() const { return _fault; }
    bool settling() const { return active() && _settling; }
    uint8_t samples() const { return _samples; }
    int32_t displacementMilliDegrees() const { return _displacementMdeg; }
    uint16_t candidateUs() const { return _candidateUs; }
    uint16_t leftStartUs() const { return _leftStartUs; }
    uint16_t rightStartUs() const { return _rightStartUs; }
    uint16_t requestedPulseUs() const {
        return active() && !_settling ? _candidateUs : _neutralUs;
    }

private:
    int32_t directedDisplacement() const
    {
        return _state == State::Left ? -_displacementMdeg : _displacementMdeg;
    }

    void resetObservation(uint32_t now)
    {
        _windowStartedMs = now;
        _windowSum = 0;
        _windowSamples = 0;
        _windows = 0;
        _samples = 0;
        _haveMotionBaseline = false;
        _displacementMdeg = 0;
    }

    void observeHeading(uint32_t now, int32_t panMdeg)
    {
        _lastSampleAtMs = now;
        if (_windowSamples == 0)
            _windowMin = _windowMax = panMdeg;
        if (panMdeg < _windowMin)
            _windowMin = panMdeg;
        if (panMdeg > _windowMax)
            _windowMax = panMdeg;
        _windowSum += panMdeg;
        ++_windowSamples;
        if (now - _windowStartedMs < HEADING_WINDOW_MS)
            return;

        if (_windowSamples < 3)
        {
            resetObservation(now);
            return;
        }
        const int32_t mean = static_cast<int32_t>(_windowSum / _windowSamples);
        if (_settling)
        {
            if (_windows == 4)
            {
                for (uint8_t i = 1; i < 4; ++i)
                {
                    _means[i - 1] = _means[i];
                    _minima[i - 1] = _minima[i];
                    _maxima[i - 1] = _maxima[i];
                }
                --_windows;
            }
            _means[_windows] = mean;
            _minima[_windows] = _windowMin;
            _maxima[_windows] = _windowMax;
            ++_windows;
        }
        else if (!_haveMotionBaseline)
        {
            _motionBaselineMdeg = _previousMeanMdeg = mean;
            _haveMotionBaseline = true;
        }
        else
        {
            const int32_t progress = _state == State::Left
                ? _previousMeanMdeg - mean : mean - _previousMeanMdeg;
            _samples = progress >= MIN_PROGRESS_MDEG
                ? (_samples < 3 ? _samples + 1 : 3) : 0;
            _previousMeanMdeg = mean;
            _displacementMdeg = mean - _motionBaselineMdeg;
            if (directedDisplacement() < -NOISE_BAND_MDEG)
                fail(Fault::WrongDirection);
        }
        _windowStartedMs = now;
        _windowSamples = 0;
        _windowSum = 0;
    }

    bool headingSettled() const
    {
        if (_windows < 4)
            return false;
        int32_t minimum = _minima[0], maximum = _maxima[0];
        for (uint8_t i = 1; i < 4; ++i)
        {
            if (_minima[i] < minimum)
                minimum = _minima[i];
            if (_maxima[i] > maximum)
                maximum = _maxima[i];
        }
        const int32_t trend = (_means[2] + _means[3] - _means[0] - _means[1]) / 2;
        return maximum - minimum <= NOISE_BAND_MDEG
            && trend >= -SETTLE_TREND_MDEG && trend <= SETTLE_TREND_MDEG;
    }

    void settle(uint32_t now, uint32_t sequence)
    {
        _settling = true;
        _outputObserved = false;
        _phaseStartedMs = now;
        _lastSequence = sequence;
        _lastSampleAtMs = now;
        resetObservation(now);
    }

    State _state = State::Idle;
    Fault _fault = Fault::None;
    uint16_t _neutralUs = AatConfigDefaults::PAN_SERVO_NEUTRAL_US;
    uint16_t _leftLimitUs = 0, _rightLimitUs = 0, _candidateUs = 0;
    uint16_t _leftStartUs = 0, _rightStartUs = 0;
    uint32_t _startedMs = 0, _phaseStartedMs = 0, _lastSequence = 0;
    int32_t _stagePanDegrees = 0;
    uint32_t _windowStartedMs = 0, _lastSampleAtMs = 0;
    int64_t _windowSum = 0;
    uint16_t _windowSamples = 0;
    int32_t _windowMin = 0, _windowMax = 0;
    int32_t _means[4] = {}, _minima[4] = {}, _maxima[4] = {};
    int32_t _motionBaselineMdeg = 0, _previousMeanMdeg = 0, _displacementMdeg = 0;
    uint8_t _windows = 0;
    bool _haveMotionBaseline = false;
    uint8_t _samples = 0;
    bool _settling = false, _outputObserved = false, _finishing = false;
};
}
