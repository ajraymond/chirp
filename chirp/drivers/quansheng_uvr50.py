# Copyright 2020 Uriel Corfa <uriel@corfa.fr>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import struct
import time
import logging

from chirp import chirp_common, errors, util, directory, memmap
from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueInteger, RadioSettingValueList, \
    RadioSettingValueBoolean, RadioSettingValueString, \
    RadioSettings
from chirp import bitwise
from textwrap import dedent

LOG = logging.getLogger(__name__)

MEM_FORMAT = """
struct channel {
  lbcd rxfreq[4];
  lbcd txfreq[4];
  ul16 rxtone;
  ul16 txtone;
  u8 unknown1:3,
     busylock:1,
     scanadd:1,
     unknown2:3;
  u8 unknown3:2,
     power:1,
     width:1,
     unknown4:4;
  u8 signalcode;
  u8 unknown5;
};

#seekto 0x0000;
struct channel memory[128];

#seekto 0x0f60;
struct channel vfo1;
#seekto 0x0f70;
struct channel vfo2;

#seekto 0x1000;
struct {
  u8 name[8];
  u8 unknown[8];
} names[128];

#seekto 0x0E20;
struct {
  u8 mdfa;           // 0x0E20
  u8 step;           // 0x0E21
  u8 squelch;        // 0x0E22
  u8 powersave;      // 0x0E23
  u8 vox;            // 0x0E24
  u8 unknownE25;     // 0x0E25
  u8 tot;            // 0x0E26
  u8 unknownE27;     // 0x0E27
  u8 dualwatch;      // 0x0E28
  u8 unknownE29[4];  // 0x0E29 .. 2C
  u8 beep;           // 0x0E2D
  u8 lang;           // 0x0E2E
  u8 unknownE2F;     // 0x0E2F

  u8 dtmfst;         // 0x0E30
  u8 unknownE31;     // 0x0E31
  u8 chan1;          // 0x0E32
  u8 chan2;          // 0x0E33
  u8 pttlt;          // 0x0E34
  u8 pttid;          // 0x0E35
  u8 unknownE36;     // 0x0E36
  u8 mdfb;           // 0x0E37
  u8 scanresmode;    // 0x0E38
  u8 autolk;         // 0x0E39
  u8 unknnownE3A[3]; // 0x0E3A .. 3C
  u8 stbycolor;      // 0x0E3D
  u8 rxcolor;        // 0x0E3E
  u8 txcolor;        // 0x0E3F

  u8 alarm;          // 0x0E40
  u8 unknownE41;     // 0x0E41
  u8 tdrab;          // 0x0E42
  u8 ste;            // 0x0E43
  u8 rpste;          // 0x0E44
  u8 rptrl;          // 0x0E45
  u8 ponmsg;         // 0x0E46
  u8 roger;          // 0x0E47
  u8 unknownE48[4];  // 0x0E48 .. 4B
  u8 vfomr;          // 0x0E4C
  u8 locked;         // 0x0E4D
  u8 abr;            // 0x0E4E
  u8 unknownE4F;     // 0x0E4F

  u8 unknownE50[10]; // 0x0E50 .. 59
  u8 aniid[5];       // 0x0E5A .. 5E
  u8 unknownE5F;     // 0x0E5F  // This is probably supposed to remain
                                // at 0xff, as a string terminator for
                                // the ANI ID
} settings;

#seekto 0x0B00;
struct {
  u8 dtmf[5];
  u8 unused[11];
} dtmf[16];
"""

MAGIC = b"\x4b\x6b\x4e\x48\x53\x47\x30\x4e\x02"
VERSION = b"\x06\x4a\x35\x36\x30\x32\x43\xf8"

SPECIALS = {
    "VFO1": ("vfo1", -2),
    "VFO2": ("vfo2", -1),
}
NUM_CHANNELS = 128

POWER_LEVELS = [chirp_common.PowerLevel("Low",  watts=1.00),
                chirp_common.PowerLevel("High", watts=4.00)]
MODES = ["NFM", "FM"]
STEPS = [2.5, 5.0, 6.25, 10.0, 12.5, 25.0]
LANGUAGES = ["Off", "Chinese", "English"]
SCANRES_MODES = ["Skip after 5s",
                 "Skip when carrier-wave drops",
                 "Stop when found"]
COLORS = ["Off", "Green", "Purple", "Sky blue"]
DTMFST_MODES = ["Off", "DT-ST", "ANI", "DT-ST + ANI"]
PTTID_MODES = ["Off", "BOT", "EOT", "Both"]
DISPLAY_MODES = ["Frequency", "Channel no", "Name"]
ALARM_MODES = ["Tone", "Code", "Site"]
TDRAB_MODES = ["Off", "A only (upper row)", "B only (lower row)"]
VFOMR_MODES = ["VFO", "Channels"]
TOT_TIMES = [str(x) for x in range(15, 601, 15)]

DTMF_CHARS = "1234567890*#ABCD"


def _send(radio, msg):
    LOG.debug("Sending msg:\n%s" % util.hexprint(msg))
    try:
        radio.pipe.write(msg)
    except Exception as e:
        raise errors.RadioError("Serial write error: %s" % e)


def _read(radio, size):
    try:
        answer = radio.pipe.read(size)
    except Exception as e:
        raise errors.RadioError("Serial read error: %s" % e)
    LOG.debug("Recv:\n%s" % util.hexprint(answer))
    return answer


def ident(radio):
    radio.pipe.timeout = 1

    _send(radio, MAGIC)
    ack = _read(radio, 1)
    time.sleep(0.1)
    if ack != b"\x06":
        raise errors.RadioError("Radio did not respond to ident")
    _send(radio, b"\x02")
    version = _read(radio, 8)
    if version != VERSION:
        raise errors.RadioError("Unknown version: %s", version)


def _read_block(radio, start, size):
    _send(radio, b"\x06")
    ack = _read(radio, 1)
    if ack != b"\x06":
        raise errors.RadioError("Radio not ready to read")

    msg = struct.pack(">BHB", ord("R"), start, size)
    _send(radio, msg)

    header = _read(radio, 4)
    op, rstart, rsize = struct.unpack(">BHB", header)
    if op != ord('W'):
        raise errors.RadioError(
            "Wrong reply: expected 87 (W), got %s (%s)" % (op, chr(op)))
    if rstart != start:
        raise errors.RadioError(
            "Radio read started at %s, expected %s" % (rstart, start))
    data = _read(radio, rsize)
    if len(data) != size:
        raise errors.RadioError("Radio refused to send block 0x%04x" % start)
    return data


def _write_block(radio, start, block):
    size = len(block)
    msg = struct.pack(">BHB", ord("W"), start, size)
    _send(radio, msg)
    _send(radio, block)
    ack = _read(radio, 1)
    if ack != b"\x06":
        raise errors.RadioError("Radio failed to write")


def do_download(radio):
    # Check we're talking to the real deal
    ident(radio)

    status = chirp_common.Status()
    status.msg = "Cloning from UV-R50"
    status.max = 0x2000

    mm = memmap.MemoryMap(b"\x00" * 0x2000)
    for i in range(0x0000, 0x1fb0, 0x40):
        status.cur = i
        radio.status_fn(status)
        data = _read_block(radio, i, 0x40)
        mm.set(i, data)

    return mm


def do_upload(radio):
    ident(radio)

    status = chirp_common.Status()
    status.msg = "Cloning to UV-R50"
    status.max = 0x2000

    # Unclear why, but the official client reads this before writing
    # and the radio won't let us write before we read this block.
    _read_block(radio, 0x0fd0, 0x40)

    _send(radio, b"\x06")
    ack = _read(radio, 1)
    if ack != b"\x06":
        raise errors.RadioError("Radio not ready to write")

    mm = radio.get_mmap()
    for i in range(0x0000, 0x1fe0, 0x10):
        status.cur = i
        radio.status_fn(status)
        block = mm[i:i+0x10]
        _write_block(radio, i, block)

    # Get the serial port connection
    serial = radio.pipe

    # Our fake radio is just a simple upload of 1000 bytes
    # to the serial port. Do that one byte at a time, reading
    # from our memory map
    for i in range(0, 1000):
        serial.write(radio.get_mmap()[i])


@directory.register
class QuanshengUVR50Radio(chirp_common.CloneModeRadio):
    """Quansheng UV-R50 radio.

    This is probably V1 of this radio, and documentation about other
    models is sparse on the internet. As far as I can tell, UV-R50-2
    and UV-R50-CX are just different cases for the same internal
    hardware, but I only have one model and I'm not sure which one.
    """
    VENDOR = "Quansheng"
    MODEL = "UV-R50"
    BAUD_RATE = 9600
    NEEDS_COMPAT_SERIAL = False

    _chars = dict(enumerate(list(range(ord('0'), ord('9') + 1)) + list(range(ord('A'), ord('Z') + 1))))

    def _ascii2quansheng(self, asc, maxlen):
        """Converts ASCII to an encoding the R50 understands."""
        revchars = {c: int(pos) for (pos, c) in list(self._chars.items())}
        asc = asc.ljust(maxlen).upper()
        qs = []
        for i in range(maxlen):
            qs.append(revchars.get(ord(asc[i]), 0xff))
        return qs

    def _quansheng2ascii(self, qs, maxlen):
        """Converts the R50's text encoding scheme to ASCII."""
        asc = ''
        for i in range(maxlen):
            c = int(qs[i])
            if c not in self._chars:
                return asc
            asc += chr(self._chars[c])
        return asc

    def _dtmf2ascii(self, dtmf):
        asci = ""
        for i in range(5):
            c = dtmf[i]
            if c - 1 > len(DTMF_CHARS):
                return asci
            asci += DTMF_CHARS[c - 1]
        return asci

    def _ascii2dtmf(self, asci):
        dtmf = []
        for x in asci.ljust(5, "?"):
            try:
                dtmf.append(DTMF_CHARS.index(x) + 1)
            except ValueError:
                dtmf.append(0xff)
        return dtmf

    def get_settings(self):
        _set = self._memobj.settings
        _dtmf = self._memobj.dtmf
        grp = RadioSettingGroup("uvr50", "UV-R50")
        grp.append(RadioSetting("vfomr", "Operation mode",
                   RadioSettingValueList(VFOMR_MODES, VFOMR_MODES[_set.vfomr])))
        grp.append(RadioSetting("tot", "Time-out timer",
                   RadioSettingValueList(TOT_TIMES, TOT_TIMES[_set.tot])))
        grp.append(RadioSetting("squelch", "Squelch level",
                   RadioSettingValueInteger(0, 9, _set.squelch)))
        grp.append(RadioSetting(
            "vox", "VOX", RadioSettingValueInteger(0, 9, _set.vox)))
        grp.append(RadioSetting("lang", "Language",
                   RadioSettingValueList(LANGUAGES, LANGUAGES[_set.lang])))
        grp.append(RadioSetting("scanresmode", "Scan Resume Mode", RadioSettingValueList(
            SCANRES_MODES, SCANRES_MODES[_set.scanresmode])))
        grp.append(RadioSetting("roger", "Roger sound",
                   RadioSettingValueBoolean(_set.roger)))
        grp.append(RadioSetting("stbycolor", "Standby back-light color",
                   RadioSettingValueList(COLORS, COLORS[_set.stbycolor])))
        grp.append(RadioSetting("rxcolor", "Receiving back-light color",
                   RadioSettingValueList(COLORS, COLORS[_set.rxcolor])))
        grp.append(RadioSetting("txcolor", "Transmitting back-light color",
                   RadioSettingValueList(COLORS, COLORS[_set.txcolor])))
        grp.append(RadioSetting("step", "Step", RadioSettingValueList(
            [str(x) for x in STEPS], str(STEPS[_set.step]))))
        grp.append(RadioSetting("powersave", "Power saving",
                   RadioSettingValueInteger(0, 4, _set.powersave)))
        grp.append(RadioSetting("abr", "Auto backlight duration (s)",
                   RadioSettingValueInteger(0, 5, _set.abr)))
        grp.append(RadioSetting("dualwatch", "Dual watch",
                   RadioSettingValueBoolean(_set.dualwatch)))
        grp.append(RadioSetting("tdrab", "Transmit row (TDR AB)",
                   RadioSettingValueList(TDRAB_MODES, TDRAB_MODES[_set.tdrab])))
        grp.append(RadioSetting("beep", "Key press beep",
                   RadioSettingValueBoolean(_set.beep)))

        def aniid_apply(setting, obj, encode):
            value = str(setting.value)
            obj.aniid = encode(value)
        rs = RadioSetting("aniid", "ANI ID", RadioSettingValueString(
            1, 5, self._quansheng2ascii(_set.aniid, 5), charset="0123456789ABCDEF "))
        rs.set_apply_callback(
            aniid_apply, _set, lambda x: self._ascii2quansheng(x, 5))
        grp.append(rs)

        grp.append(RadioSetting("dtmfst", "DTMFST", RadioSettingValueList(
            DTMFST_MODES, DTMFST_MODES[_set.dtmfst])))
        grp.append(RadioSetting("pttid", "Send PTT ID", RadioSettingValueList(
            PTTID_MODES, PTTID_MODES[_set.pttid])))
        grp.append(RadioSetting("pttlt", "PTT ID prolong time (ms)",
                   RadioSettingValueInteger(0, 30, _set.pttlt)))
        grp.append(RadioSetting("mdfa", "Display mode channel A",
                   RadioSettingValueList(DISPLAY_MODES, DISPLAY_MODES[_set.mdfa])))
        grp.append(RadioSetting("mdfb", "Display mode channel B",
                   RadioSettingValueList(DISPLAY_MODES, DISPLAY_MODES[_set.mdfb])))
        grp.append(RadioSetting("autolk", "Auto-lock UI",
                   RadioSettingValueBoolean(_set.autolk)))
        grp.append(RadioSetting("locked", "Locked UI",
                   RadioSettingValueBoolean(_set.locked)))
        grp.append(RadioSetting("alarm", "Alarm mode", RadioSettingValueList(
            ALARM_MODES, ALARM_MODES[_set.alarm])))
        grp.append(RadioSetting("ste", "Tail tone elimination",
                   RadioSettingValueBoolean(_set.ste)))
        grp.append(RadioSetting("rpste", "Repeater tail tone elimination (ms)",
                   RadioSettingValueInteger(0, 10, _set.rpste)))
        grp.append(RadioSetting("rptrl", "Repeater tail tone elimination time (ms)",
                   RadioSettingValueInteger(0, 10, _set.rptrl)))
        grp.append(RadioSetting("ponmsg", "Display logo at startup",
                   RadioSettingValueBoolean(_set.ponmsg)))

        dtmf = RadioSettingGroup("dtmf", "DTMF")

        def dtmf_apply(setting, obj, encode):
            value = str(setting.value)
            obj.dtmf = encode(value)

        for i in range(0, 15):
            vs = RadioSettingValueString(0, 5,
                                         self._dtmf2ascii(_dtmf[i].dtmf),
                                         False)
            vs.set_charset(DTMF_CHARS)
            rs = RadioSetting("dtmf/%d" % i, "DTMF %d" % (i + 1), vs)
            rs.set_apply_callback(dtmf_apply, self._memobj.dtmf[i],
                                  self._ascii2dtmf)
            dtmf.append(rs)

        g = RadioSettings(grp, dtmf)
        return g

    def set_settings(self, settings):
        """Automatically apply the settings recursively."""
        _settings = self._memobj.settings
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
            else:
                name = element.get_name()
                if "." in name:
                    bits = name.split(".")
                    obj = self._memobj
                    for bit in bits[:-1]:
                        if "/" in bit:
                            bit, index = bit.split("/", 1)
                            index = int(index)
                            obj = getattr(obj, bit)[index]
                        else:
                            obj = getattr(obj, bit)
                    setting = bits[-1]
                else:
                    obj = _settings
                    setting = element.get_name()

                if element.has_apply_callback():
                    element.run_apply_callback()
                elif element.value.get_mutable():
                    setattr(obj, setting, element.value)

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_bank = False
        rf.has_dtcs = True
        rf.has_ctone = True
        rf.has_rx_dtcs = True
        rf.has_offset = True
        rf.has_name = True
        rf.has_cross = True
        rf.has_settings = True
        rf.has_tuning_step = False
        rf.can_odd_split = True
        rf.can_delete = True

        rf.memory_bounds = (0, NUM_CHANNELS - 1)

        rf.valid_skips = [""]  # UV-R50 has no support for scan skips
        rf.valid_bands = [(136000000, 174000000),
                          (400000000, 520000000),
                          ]
        rf.valid_duplexes = ["", "-", "+", "off", "split"]
        rf.valid_characters = "".join([chr(x) for x in list(self._chars.values())])
        rf.valid_name_length = 7
        rf.valid_modes = MODES
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = ["Tone->Tone", "Tone->DTCS",
                                "DTCS->Tone", "DTCS->DTCS", "DTCS->",
                                "->Tone", "->DTCS"]
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_tuning_steps = STEPS
        rf.valid_special_chans = list(SPECIALS.keys())
        return rf

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    def sync_in(self):
        self._mmap = do_download(self)
        self.process_mmap()

    def sync_out(self):
        do_upload(self)

    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number])

    def get_memory(self, number):
        mem = chirp_common.Memory()

        if number in SPECIALS:
            name = number
            _mem = self._memobj.__getattr__(SPECIALS[number][0])
            mem.extd_number = number
            number = SPECIALS[number][1]
        else:
            _nam = self._memobj.names[number]
            name = self._quansheng2ascii(_nam.name, 8)
            _mem = self._memobj.memory[number]

        mem.number = number

        if _mem.rxfreq.get_raw() in ["\xff\xff\xff\xff", "\xa5\xa5\xa5\xa5"]:
            mem.empty = True
            return mem

        mem.name = name
        mem.power = POWER_LEVELS[_mem.power]
        mem.mode = MODES[_mem.width]
        mem.freq = int(_mem.rxfreq) * 10
        offset = (int(_mem.txfreq) - int(_mem.rxfreq)) * 10

        if offset == 0:
            mem.duplex = "off"
            mem.offset = 0
        elif offset > 120000000:  # 120MHz
            mem.duplex = "split"
            mem.offset = _mem.txfreq * 10
        elif offset < 0:
            mem.duplex = "-"
            mem.offset = abs(offset)
        else:
            mem.duplex = "+"
            mem.offset = offset

        class ToneWord(object):
            def __init__(self, word):
                """Decodes the tx or rx "tone" word of a channel or freq.

                The encoding scheme varies depend on what is encoded:

                - The most significant nibble encodes what kind of
                  tone/code squelch is used (CTCSS or DTCS);

                - For CTCSS, the last 3 nibbles encode the tone as a
                  straight frequency;

                - For DTCS, the last 2 nibbles (lower byte) encode the
                  DTCS code, and the polarity is encoded in the most
                  significant bit of the word.

                If this channel is not set to tx/rx anything, the high
                byte is 0xff.
                """
                self.is_set = (word & 0xff00 != 0xff00) and word != 0x0000
                self.is_dtcs = word & 0x2000 and self.is_set
                if self.is_dtcs:
                    self.dtcs_pol = "R" if word & 0x8000 else "N"
                    self.dtcs_index = (word & 0x01ff)
                else:
                    self.dtcs_pol = "N"
                    self.dtcs_index = None
                self.is_ctcss = not self.is_dtcs and self.is_set
                self.tone = (word & 0x0fff) if self.is_ctcss else None
                if self.is_ctcss:
                    self.mode = "Tone"
                elif self.is_dtcs:
                    self.mode = "DTCS"
                else:
                    self.mode = ""

            def equals(self, o):
                if self.is_set != o.is_set:
                    return False
                if self.mode != o.mode:
                    return False
                if self.mode == "Tone" and self.tone != o.tone:
                    return False
                # For DTCS, we consider different polarities as
                # "equal", i.e. that's not a "Cross" mode.
                if self.mode == "DTCS" and self.dtcs_index != o.dtcs_index:
                    return False
                return True

        rx = ToneWord(_mem.rxtone)
        tx = ToneWord(_mem.txtone)

        # The logic here gets a bit complicated, because this radio
        # lets you do any kind of crossing and just stores "what do I
        # need to send" and "what do I expect to receive" for each
        # channel. Chirp likes symmetry more, e.g. TSQL is
        # "Tone->Tone" when the tones are equal. The wiki page at
        # https://chirpmyradio.com/projects/chirp/wiki/DevelopersToneModes
        # has an overview of Chirp's tone modes.
        # The reverse logic in set_memory is maybe easier to follow.
        if tx.equals(rx):
            if tx.is_ctcss:
                mem.rtone = float(int(tx.tone)) / 10.0
                mem.ctone = float(int(tx.tone)) / 10.0
                mem.tmode = "TSQL"
            elif tx.is_dtcs:
                mem.dtcs = chirp_common.ALL_DTCS_CODES[tx.dtcs_index]
                mem.rx_dtcs = chirp_common.ALL_DTCS_CODES[tx.dtcs_index]
                mem.tmode = "DTCS"
            else:
                mem.tmode = ""
        elif tx.is_ctcss and not rx.is_set:
            mem.tmode = "Tone"
            mem.rtone = float(int(tx.tone)) / 10.0
        else:
            mem.tmode = "Cross"
            mem.cross_mode = "%s->%s" % (tx.mode, rx.mode)
            if rx.is_ctcss:
                mem.ctone = float(int(rx.tone)) / 10.0
            elif rx.is_dtcs:
                mem.rx_dtcs = chirp_common.ALL_DTCS_CODES[rx.dtcs_index]
            if tx.is_ctcss:
                mem.rtone = float(int(tx.tone)) / 10.0
            elif tx.is_dtcs:
                mem.dtcs = chirp_common.ALL_DTCS_CODES[tx.dtcs_index]

        mem.dtcs_polarity = "%s%s" % (tx.dtcs_pol, rx.dtcs_pol)

        mem.extra = RadioSettingGroup("Extra", "extra")
        mem.extra.append(RadioSetting("busylock", "Busy lock",
                                      RadioSettingValueBoolean(_mem.busylock)))
        mem.extra.append(RadioSetting("scanadd", "Scan add",
                                      RadioSettingValueBoolean(_mem.scanadd)))

        if mem.freq == 0:
            mem.empty = True

        return mem

    def set_memory(self, mem):
        # Get a low-level memory object mapped to the image
        if mem.number < 0:
            _mem = self._memobj.__getattr__(SPECIALS[mem.extd_number][0])
        else:
            _mem = self._memobj.memory[mem.number]
            _nam = self._memobj.names[mem.number]
            _nam.name = self._ascii2quansheng(mem.name, 8)

        if mem.empty:
            _mem.set_raw(b'\xa5' * 8 + b'\xff' * 8)
            return

        # Convert to low-level frequency representation
        _mem.rxfreq = mem.freq / 10
        if mem.duplex in [None, "", "off"]:
            _mem.txfreq = _mem.rxfreq
        elif mem.duplex == "+":
            _mem.txfreq = (mem.freq + mem.offset) / 10
        elif mem.duplex == "-":
            _mem.txfreq = (mem.freq - mem.offset) / 10
        else:  # "split", aka odd split
            _mem.txfreq = mem.offset / 10

        _mem.power = (mem.power != POWER_LEVELS[0])
        _mem.width = (mem.mode == "FM")

        def encodetone(mode, tone, dtcs, polarity):
            """This is the reverse of decodetone in get_memory."""
            if mode == "":
                return 0xffff
            elif mode == "Tone":
                return 0x0fff & int(tone * 10)
            elif mode == "DTCS":
                return (
                    (0x8000 if polarity == "R" else 0x0000) |
                    (0x2000) |
                    (0x0fff & (chirp_common.ALL_DTCS_CODES.index(dtcs)))
                )

        if mem.tmode == "Cross":
            txmode, rxmode = mem.cross_mode.split("->")
            _mem.txtone = encodetone(txmode, mem.rtone,
                                     mem.dtcs, mem.dtcs_polarity[0])
            _mem.rxtone = encodetone(rxmode, mem.ctone,
                                     mem.rx_dtcs, mem.dtcs_polarity[1])
        elif mem.tmode == "Tone":
            _mem.txtone = encodetone("Tone", mem.rtone,
                                     mem.dtcs, mem.dtcs_polarity[0])
            _mem.rxtone = 0xffff
        elif mem.tmode == "TSQL":
            _mem.txtone = encodetone("Tone", mem.ctone,
                                     mem.dtcs, mem.dtcs_polarity[0])
            _mem.rxtone = encodetone("Tone", mem.ctone,
                                     mem.dtcs, mem.dtcs_polarity[1])
        elif mem.tmode == "DTCS":
            _mem.txtone = encodetone("DTCS", mem.rtone,
                                     mem.dtcs, mem.dtcs_polarity[0])
            _mem.rxtone = encodetone("DTCS", mem.ctone,
                                     mem.dtcs, mem.dtcs_polarity[1])
        else:
            _mem.txtone = 0xffff
            _mem.rxtone = 0xffff

        # TODO: we may want to work around issue 4121: settings is
        # empty when editing in the "table" interface. See UV5R driver
        # for a working approach.
        for setting in mem.extra:
            setattr(_mem, setting.get_name(), setting.value)
