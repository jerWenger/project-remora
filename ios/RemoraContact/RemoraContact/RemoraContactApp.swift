//  RemoraContact
//  Sends this phone's GPS fixes to a MOOS shoreside (iContactUDP) so the
//  vessel carrying the phone shows up as a contact for the boat's helm.

import SwiftUI

@main
struct RemoraContactApp: App {
    @StateObject private var sender = FixSender()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(sender)
        }
    }
}
