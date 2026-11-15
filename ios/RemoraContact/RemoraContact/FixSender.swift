//  FixSender.swift
//  CoreLocation in, one UDP datagram per fix out.
//
//  Datagram (what iContactUDP on the shoreside expects), one line:
//    NAME=skiff,LAT=42.3584310,LON=-71.0875120,SPD=1.42,HDG=268.5,TIME=1794772478.31,ACC=4.1
//  SPD is m/s over ground, HDG is the course over ground in degrees true or -1
//  when the phone has none (it only has a course while moving), TIME is the
//  unix time of the fix, ACC its horizontal accuracy in metres.

import Combine
import CoreLocation
import Foundation
import Network

final class FixSender: NSObject, ObservableObject, CLLocationManagerDelegate {
    @Published private(set) var running = false
    @Published private(set) var authorization: CLAuthorizationStatus = .notDetermined
    @Published private(set) var lastFix: CLLocation?
    @Published private(set) var sentCount = 0
    @Published private(set) var lastSent: Date?
    @Published private(set) var linkState = "idle"
    @Published private(set) var lastError: String?

    private let manager = CLLocationManager()
    private var connection: NWConnection?
    private var heartbeat: Timer?
    private var name = "skiff"

    override init() {
        super.init()
        manager.delegate = self
        manager.desiredAccuracy = kCLLocationAccuracyBestForNavigation
        manager.distanceFilter = kCLDistanceFilterNone
        manager.activityType = .otherNavigation
        // Keep fixes coming with the screen off or the app in the background.
        manager.pausesLocationUpdatesAutomatically = false
        authorization = manager.authorizationStatus
    }

    /// Start sending to host:port as the vessel `name`.
    func start(name: String, host: String, port: Int) {
        let cleanName = name.lowercased().filter { $0.isLetter || $0.isNumber || $0 == "_" }
        guard !cleanName.isEmpty else { lastError = "Name needs letters or digits"; return }
        guard !host.isEmpty, let nwPort = NWEndpoint.Port(rawValue: UInt16(clamping: port)), port > 0 else {
            lastError = "Host or port is not valid"
            return
        }
        stop()
        self.name = cleanName
        lastError = nil
        sentCount = 0

        let conn = NWConnection(host: NWEndpoint.Host(host), port: nwPort, using: .udp)
        conn.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                switch state {
                case .ready: self?.linkState = "ready"
                case .waiting(let error): self?.linkState = "waiting: \(error.localizedDescription)"
                case .failed(let error): self?.linkState = "failed: \(error.localizedDescription)"
                case .cancelled: self?.linkState = "idle"
                default: self?.linkState = "connecting"
                }
            }
        }
        conn.start(queue: .main)
        connection = conn

        if manager.authorizationStatus == .notDetermined {
            manager.requestWhenInUseAuthorization()
        }
        manager.allowsBackgroundLocationUpdates = true
        manager.showsBackgroundLocationIndicator = true
        manager.startUpdatingLocation()
        running = true

        // CoreLocation goes quiet when the phone is not moving. Repeat the
        // last fix once a second so the contact does not go stale ashore.
        heartbeat = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            guard let self, let fix = self.lastFix, let sent = self.lastSent else { return }
            if Date().timeIntervalSince(sent) > 1.5, Date().timeIntervalSince(fix.timestamp) < 15 {
                self.send(fix)
            }
        }
    }

    func stop() {
        heartbeat?.invalidate()
        heartbeat = nil
        manager.stopUpdatingLocation()
        manager.allowsBackgroundLocationUpdates = false
        connection?.cancel()
        connection = nil
        running = false
    }

    // MARK: CLLocationManagerDelegate

    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        authorization = manager.authorizationStatus
    }

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        // Use the newest fix; skip invalid ones (negative accuracy).
        guard let fix = locations.last, fix.horizontalAccuracy >= 0 else { return }
        lastFix = fix
        if running { send(fix) }
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        lastError = "Location: \(error.localizedDescription)"
    }

    // MARK: Sending

    static func datagram(name: String, fix: CLLocation) -> String {
        let speed = max(0, fix.speed)                    // -1 = unknown
        let course = fix.course >= 0 ? fix.course : -1   // -1 = no course (standing still)
        return String(format: "NAME=%@,LAT=%.7f,LON=%.7f,SPD=%.2f,HDG=%.1f,TIME=%.2f,ACC=%.1f",
                      name, fix.coordinate.latitude, fix.coordinate.longitude,
                      speed, course, fix.timestamp.timeIntervalSince1970, fix.horizontalAccuracy)
    }

    private func send(_ fix: CLLocation) {
        guard let connection else { return }
        let line = Self.datagram(name: name, fix: fix)
        connection.send(content: line.data(using: .utf8), completion: .contentProcessed { [weak self] error in
            DispatchQueue.main.async {
                if let error {
                    self?.lastError = "Send: \(error.localizedDescription)"
                } else {
                    self?.sentCount += 1
                    self?.lastSent = Date()
                }
            }
        })
    }
}
