# Narrow the scan around a contact and lead shots from recent sightings.
# Clunkyard guide: https://clunkyard.com/sdk/python
import math

import clunkyard as cy

SEARCH_TURN = 0.33  # enough to turn against the brake in the force rulesets
SEARCH_ARC = 40  # degrees: sweeps the circle in half a second
HOLD_WIDTH = 60  # units across at the target: it stays in the beam from tick to tick
MIN_ARC = 6  # degrees: the narrowest the beam holds
WINDOW = 12  # ticks of sightings the aim averages
SIGHTINGS = 8  # narrow sightings in the window before a shot
LOOK_EVERY = 40  # ticks between looks around the target, for a nearer robot
REACQUIRE = 10  # ticks to look again where a lost target was


def steer(towards, current):
    """A turn or pitch-turn control towards an angle, full at 45 degrees off."""
    return cy.angles.clamp(cy.angles.angle_diff(towards, current) / 45)


class Starter(cy.Robot):
    def on_start(self):
        # Flying free, robots may be above or below.
        self.free = self.match.is_3d and self.arena.gravity.z == 0
        self.dodges, self.dodge_end, self.slot = 0, 0, 0

    def fit(self, slot, tick):
        """A straight line through the narrowest of the last WINDOW ticks'
        sightings: where the robot is now, how fast it moves, and how many
        sightings it rests on. Each sighting is off by its bearing_error; the
        line averages that out."""
        recent = [s for s in self.tracker.history(slot) if s.tick > tick - WINDOW]
        if len(recent) < 2:
            return None
        best = min(s.bearing_error for s in recent)
        seen = [s for s in recent if s.bearing_error <= 1.5 * best]
        n = len(seen)
        if n < 2:
            return None
        t0 = sum(s.tick for s in seen) / n
        spread = sum((s.tick - t0) ** 2 for s in seen)
        position, velocity = [], []
        for axis in ("x", "y", "z"):
            mean = sum(getattr(s.position, axis) for s in seen) / n
            slope = sum((s.tick - t0) * (getattr(s.position, axis) - mean) for s in seen) / spread
            position.append(mean + slope * (tick - t0))
            velocity.append(slope * self.match.tick_rate)
        return cy.Vec3(*position), cy.Vec3(*velocity), n

    def on_turn(self):
        me, gun, scanner, tick = self.me, self.gun, self.scanner, self.clock.tick
        self.drive.forces(brake=0.2)
        target = self.sensors.nearest
        cue = self.tracker.cue  # the freshest passive contact, for a few seconds
        last_seen = self.tracker.last_seen(self.slot) if self.slot else None
        if target:
            self.slot = target.slot
            throttle = 0.7 if target.range > 350 else -0.4 if target.range < 200 else 0.2
            self.drive.set(
                throttle=throttle,
                turn=steer(target.bearing, me.heading),
                pitch_turn=steer(target.elevation, me.pitch),
            )
            # Halve the beam each tick until it is HOLD_WIDTH across at the target:
            # narrow enough for a precise bearing, wide enough to keep it in.
            hold = max(MIN_ARC, math.degrees(2 * math.atan2(HOLD_WIDTH / 2, target.range)))
            # In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks,
            # widen it for a tick: a nearer robot it finds becomes the target.
            arc = (
                SEARCH_ARC
                if tick % LOOK_EVERY == 0
                else max(hold, min(SEARCH_ARC, scanner.last.arc / 2))
            )
            scanner.set(heading=target.bearing, arc=arc, pitch=target.elevation)
            # The heaviest shot that still covers the range in about a second,
            # aimed where the target will be, lobbed under gravity.
            power = gun.power_for_flight_time(target.range, 1)
            # An exact contact: gun.lead is the intercept if the target holds
            # course. A noisy one: lead the fitted line, and fire only once it
            # rests on enough sightings.
            ready = True
            if target.bearing_error > 0:
                fit = self.fit(target.slot, tick)
                aim = gun.lead_point(fit[0], fit[1], power) if fit else None
                ready = fit is not None and fit[2] >= SIGHTINGS
            else:
                aim = gun.lead(target, power)
            heading, pitch = (aim.heading, aim.pitch) if aim else (target.bearing, target.elevation)
            gun.aim(heading, pitch)
            now = gun.aim_now
            on_target = (
                abs(cy.angles.angle_diff(heading, now.heading)) < 3 and abs(pitch - now.pitch) < 3
            )
            if ready and on_target and gun.can_fire(power, 10):
                gun.fire(power)
            # A line for your trace, which only you see: uncomment to follow the aim.
            # self.debug.log(f"range {target.range:.0f} aim off {cy.angles.angle_diff(heading, now.heading):.1f}")
            self.status.set("TRACK" if ready else "NARROW", target.range)
        elif last_seen is not None and tick - last_seen <= REACQUIRE:
            # Lost it: look again where it was last seen, widening the beam, before searching.
            where = self.tracker.position(self.slot)
            bearing = cy.angles.bearing_to(me.position, where)
            elevation = cy.angles.elevation_to(me.position, where) if self.free else 0
            self.drive.set(
                throttle=0.5, turn=steer(bearing, me.heading), pitch_turn=steer(elevation, me.pitch)
            )
            scanner.set(heading=bearing, arc=min(SEARCH_ARC, scanner.last.arc * 2), pitch=elevation)
            self.status.set("REACQUIRE")
        elif cue:
            # Head for where it was heard, sweeping an arc that still reaches
            # it across the bearing's error (and, flying free, the elevation's).
            bearing = cy.angles.bearing_to(me.position, cue.position)
            distance = cy.angles.distance(me.position, cue.position)
            elevation = cy.angles.elevation_to(me.position, cue.position) if self.free else 0
            error = min(cue.bearing_error, 30)
            across = (tick % 5 - 2) / 2 * error
            up = ((tick // 5) % 3 - 1) * error / 2 if self.free else 0
            self.drive.set(
                throttle=0.7, turn=steer(bearing, me.heading), pitch_turn=steer(elevation, me.pitch)
            )
            scanner.set(
                heading=bearing + across,
                arc=scanner.arc_for_reach(distance + 100),
                pitch=elevation + up,
            )
            self.status.set("CUE", distance)
        else:
            self.drive.set(throttle=0.5, turn=SEARCH_TURN, pitch_turn=steer(0, me.pitch))
            # Sweeps the circle in half a second, flying free at three heights in turn.
            pitch = ((tick // 10) % 3 - 1) * 35 if self.free else 0
            scanner.set(
                heading=cy.angles.normalize(scanner.last.heading + 36), arc=SEARCH_ARC, pitch=pitch
            )
            self.status.set("SEARCH")
        # A simple dodge: vary it, or predict the shooter.
        if tick >= self.dodge_end and any(c.source == "muzzle" for c in self.sensors.cues):
            self.dodges += 1
            self.dodge_end = tick + 10
        if tick < self.dodge_end:
            # Off the line of fire: left, then right next time (turn is positive left, strafe right).
            left = self.dodges % 2 == 1
            if self.match.control == "forces":
                self.drive.set(throttle=0.5)
                self.drive.forces(strafe=-1 if left else 1)
            else:
                self.drive.set(throttle=1, turn=1 if left else -1)
            self.status.set("dodge:left" if left else "dodge:right", self.dodges)
        # Low battery: drive gently so the shots can still be paid for.
        if self.battery.charge < self.battery.capacity / 4:
            self.drive.set(throttle=self.drive.get("throttle") * 0.5)
        # The storm hurts inside: near its edge, drive straight away from its centre.
        storm = self.nav.storm
        if storm and storm.margin(me.position) > -60:
            away = cy.angles.bearing_to(storm.center, me.position)
            self.drive.set(
                throttle=0.8, turn=cy.angles.clamp(cy.angles.angle_diff(away, me.heading) / 30)
            )
        # Sudden death: head back inside the safe zone.
        zone = self.nav.zone
        if zone and zone.margin(me.position) < 60:
            climb = (
                cy.angles.elevation_to(me.position, zone.center) if zone.shape == "sphere" else 0
            )
            self.drive.set(
                throttle=0.8,
                turn=cy.angles.clamp(
                    cy.angles.angle_diff(cy.angles.bearing_to(me.position, zone.center), me.heading)
                    / 30
                ),
                pitch_turn=cy.angles.clamp((climb - me.pitch) / 30),
            )


if __name__ == "__main__":
    cy.run(Starter())
