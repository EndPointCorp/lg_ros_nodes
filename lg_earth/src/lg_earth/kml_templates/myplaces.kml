<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"
    xmlns:gx="http://www.google.com/kml/ext/2.2"
    xmlns:kml="http://www.opengis.net/kml/2.2"
    xmlns:atom="http://www.w3.org/2005/Atom">
    <!--// This file is a FreeMarker-processed template, created by Interactive Spaces //-->
    <Document>
        <Folder>
            <name>My Places</name>
            <open>1</open>
            <Style>
                <ListStyle>
                    <listItemType>check</listItemType>
                    <ItemIcon>
                        <state>open</state>
                        <href>:/mysavedplaces_open.png</href>
                    </ItemIcon>
                    <ItemIcon>
                        <state>closed</state>
                        <href>:/mysavedplaces_closed.png</href>
                    </ItemIcon>
                    <bgColor>00ffffff</bgColor>
                    <maxSnippetLines>2</maxSnippetLines>
                </ListStyle>
            </Style>
            <Folder>
                <name>KML Sync</name>
                <NetworkLink>
                    <name>Master KML CMS</name>
                    <Link>
                        <href>${ge.kml.syncBase}/master.kml</href>
                    </Link>
                </NetworkLink>
                <NetworkLink>
                    <name>KML Update CMS</name>
                    <Link>
                        <href>${ge.kml.syncBase}/network_link_update.kml?window_slug=${ge.window.name}</href>
                        <refreshMode>onInterval</refreshMode>
                        <refreshInterval>1</refreshInterval>
                        <viewRefreshMode>onStop</viewRefreshMode>
                        <viewRefreshTime>1</viewRefreshTime>
                        <viewFormat>bboxWest=[bboxWest]&amp;bboxSouth=[bboxSouth]&amp;bboxEast=[bboxEast]&amp;bboxNorth=[bboxNorth]&amp;lookatLon=[lookatLon]&amp;lookatLat=[lookatLat]&amp;lookatRange=[lookatRange]&amp;lookatTilt=[lookatTilt]&amp;lookatHeading=[lookatHeading]&amp;cameraLon=[cameraLon]&amp;cameraLat=[cameraLat]&amp;cameraAlt=[cameraAlt]</viewFormat>
                    </Link>
                </NetworkLink>
            </Folder>
        </Folder>
    </Document>
</kml>
