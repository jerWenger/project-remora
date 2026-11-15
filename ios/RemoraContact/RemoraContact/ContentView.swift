//  ContentView.swift

import CoreLocation
import SwiftUI

struct ContentView: View {
    @EnvironmentObject private var sender: FixSender

    // The shoreside laptop's Tailscale address and iContactUDP's port.
    @AppStorage("vesselName") private var vesselName = "skiff"
    @AppStorage("host") private var host = "100.124.86.114"
    @AppStorage("port") private var port = 9300
    @AppStorage("keepAwake") private var keepAwake = true

    var body: some View {
        NavigationStack {
            Form {
                Section("This vessel") {
                    LabeledContent("Name") {
                        TextField("skiff", text: $vesselName)
                            .multilineTextAlignment(.trailing)
                            .textInputAutocapitalization(.never)
                            .autocorrectionDisabled()
                    }
                }
                .disabled(sender.running)

                Section {
                    LabeledContent("Host") {
                        TextField("100.x.y.z", text: $host)
                            .multilineTextAlignment(.trailing)
                            .keyboardType(.numbersAndPunctuation)
                            .textInputAutocapitalization(.never)
                            .autocorrectionDisabled()
                    }
                    LabeledContent("UDP port") {
                        TextField("9300", value: $port, format: .number.grouping(.never))
                            .multilineTextAlignment(.trailing)
                            .keyboardType(.numberPad)
                    }
                } header: {
                    Text("Shoreside")
                } footer: {
                    Text("The laptop running the MOOS shoreside, by its Tailscale address. Tailscale must be connected on this phone.")
                }
                .disabled(sender.running)

                Section {
                    Button(sender.running ? "Stop sending" : "Start sending") {
                        if sender.running {
                            sender.stop()
                        } else {
                            sender.start(name: vesselName, host: host, port: port)
                        }
                    }
                    .frame(maxWidth: .infinity)
                    .font(.headline)
                    .tint(sender.running ? .red : .accentColor)

                    Toggle("Keep screen on", isOn: $keepAwake)
                }

                Section("Status") {
                    LabeledContent("Location access", value: authorizationText)
                    LabeledContent("Link", value: sender.linkState)
                    LabeledContent("Fixes sent", value: "\(sender.sentCount)")
                    if let fix = sender.lastFix {
                        LabeledContent("Position", value: String(format: "%.5f, %.5f", fix.coordinate.latitude, fix.coordinate.longitude))
                        LabeledContent("Accuracy", value: String(format: "%.0f m", fix.horizontalAccuracy))
                        LabeledContent("Speed", value: String(format: "%.1f m/s", max(0, fix.speed)))
                        LabeledContent("Course", value: fix.course >= 0 ? String(format: "%.0f°", fix.course) : "none (not moving)")
                    } else {
                        LabeledContent("Position", value: "no fix yet")
                    }
                    if let error = sender.lastError {
                        Text(error).foregroundStyle(.red)
                    }
                }
            }
            .navigationTitle("Remora Contact")
        }
        .onAppear { applyKeepAwake() }
        .onChange(of: keepAwake) { applyKeepAwake() }
        .onChange(of: sender.running) { applyKeepAwake() }
    }

    private var authorizationText: String {
        switch sender.authorization {
        case .authorizedAlways: return "always"
        case .authorizedWhenInUse: return "while in use"
        case .denied: return "denied (Settings > Privacy)"
        case .restricted: return "restricted"
        default: return "not asked yet"
        }
    }

    private func applyKeepAwake() {
        UIApplication.shared.isIdleTimerDisabled = keepAwake && sender.running
    }
}

#Preview {
    ContentView().environmentObject(FixSender())
}
