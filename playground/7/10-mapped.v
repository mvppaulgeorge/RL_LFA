// Benchmark "adder" written by ABC on Wed Jul 17 15:34:27 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n336, new_n338,
    new_n339, new_n341, new_n343, new_n344, new_n346, new_n347, new_n348,
    new_n350, new_n351;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand42aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n05x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor022aa1n08x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n06x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oaih12aa1n06x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nor022aa1n06x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand02aa1n04x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nona23aa1n03x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  tech160nm_fixnrc02aa1n05x5   g019(.a(\b[7] ), .b(\a[8] ), .out0(new_n115));
  xnrc02aa1n12x5               g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  nor043aa1n04x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  tech160nm_fiaoi012aa1n05x5   g022(.a(new_n112), .b(new_n110), .c(new_n113), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[8] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[7] ), .o1(new_n120));
  nor042aa1n04x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  oao003aa1n06x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .carry(new_n122));
  inv000aa1n02x5               g027(.a(new_n122), .o1(new_n123));
  oai013aa1n06x5               g028(.a(new_n123), .b(new_n116), .c(new_n115), .d(new_n118), .o1(new_n124));
  nand42aa1n03x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n97), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n127));
  nor042aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n04x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  nano23aa1n06x5               g036(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n133));
  tech160nm_fiaoi012aa1n05x5   g038(.a(new_n128), .b(new_n97), .c(new_n129), .o1(new_n134));
  nor022aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n03x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aobi12aa1n06x5               g042(.a(new_n137), .b(new_n133), .c(new_n134), .out0(new_n138));
  aoi112aa1n02x5               g043(.a(new_n137), .b(new_n128), .c(new_n129), .d(new_n97), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n138), .b(new_n133), .c(new_n139), .o1(\s[11] ));
  nor022aa1n08x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  obai22aa1n02x7               g047(.a(new_n142), .b(new_n141), .c(new_n138), .d(new_n135), .out0(new_n143));
  nona22aa1n02x4               g048(.a(new_n142), .b(new_n141), .c(new_n135), .out0(new_n144));
  oai012aa1n02x5               g049(.a(new_n143), .b(new_n144), .c(new_n138), .o1(\s[12] ));
  nona23aa1n03x5               g050(.a(new_n142), .b(new_n136), .c(new_n135), .d(new_n141), .out0(new_n146));
  nano22aa1n03x7               g051(.a(new_n146), .b(new_n126), .c(new_n130), .out0(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n148));
  nano23aa1n06x5               g053(.a(new_n135), .b(new_n141), .c(new_n142), .d(new_n136), .out0(new_n149));
  aboi22aa1n03x5               g054(.a(new_n134), .b(new_n149), .c(new_n144), .d(new_n142), .out0(new_n150));
  nor002aa1d32x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nand42aa1d28x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n152), .b(new_n151), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n148), .c(new_n150), .out0(\s[13] ));
  inv000aa1d42x5               g059(.a(new_n151), .o1(new_n155));
  norb02aa1n03x5               g060(.a(new_n104), .b(new_n103), .out0(new_n156));
  norb02aa1n03x5               g061(.a(new_n106), .b(new_n105), .out0(new_n157));
  nanb03aa1n06x5               g062(.a(new_n102), .b(new_n157), .c(new_n156), .out0(new_n158));
  norb02aa1n02x5               g063(.a(new_n111), .b(new_n110), .out0(new_n159));
  nanb02aa1n02x5               g064(.a(new_n112), .b(new_n113), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n116), .o1(new_n161));
  nona23aa1n03x5               g066(.a(new_n161), .b(new_n159), .c(new_n115), .d(new_n160), .out0(new_n162));
  norp03aa1n03x5               g067(.a(new_n115), .b(new_n116), .c(new_n118), .o1(new_n163));
  nor002aa1n03x5               g068(.a(new_n163), .b(new_n122), .o1(new_n164));
  aoai13aa1n04x5               g069(.a(new_n164), .b(new_n162), .c(new_n158), .d(new_n108), .o1(new_n165));
  oai012aa1n02x5               g070(.a(new_n142), .b(new_n141), .c(new_n135), .o1(new_n166));
  tech160nm_fioai012aa1n05x5   g071(.a(new_n166), .b(new_n146), .c(new_n134), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n153), .b(new_n167), .c(new_n165), .d(new_n147), .o1(new_n168));
  nor022aa1n16x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1d28x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n168), .c(new_n155), .out0(\s[14] ));
  nano23aa1d15x5               g077(.a(new_n151), .b(new_n169), .c(new_n170), .d(new_n152), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n167), .c(new_n165), .d(new_n147), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n173), .o1(new_n175));
  aoi012aa1n03x5               g080(.a(new_n169), .b(new_n151), .c(new_n170), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n175), .c(new_n148), .d(new_n150), .o1(new_n177));
  nor042aa1n06x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand42aa1n06x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n03x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aoi112aa1n02x5               g085(.a(new_n180), .b(new_n169), .c(new_n170), .d(new_n151), .o1(new_n181));
  aoi022aa1n02x5               g086(.a(new_n177), .b(new_n180), .c(new_n174), .d(new_n181), .o1(\s[15] ));
  inv040aa1n02x5               g087(.a(new_n180), .o1(new_n183));
  aoi012aa1n02x7               g088(.a(new_n183), .b(new_n174), .c(new_n176), .o1(new_n184));
  nor042aa1n03x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  tech160nm_finand02aa1n03p5x5 g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  nanb02aa1n03x5               g091(.a(new_n185), .b(new_n186), .out0(new_n187));
  aoai13aa1n02x5               g092(.a(new_n187), .b(new_n178), .c(new_n177), .d(new_n180), .o1(new_n188));
  nona22aa1n02x4               g093(.a(new_n186), .b(new_n185), .c(new_n178), .out0(new_n189));
  oai012aa1n02x5               g094(.a(new_n188), .b(new_n184), .c(new_n189), .o1(\s[16] ));
  nano23aa1n02x5               g095(.a(new_n178), .b(new_n185), .c(new_n186), .d(new_n179), .out0(new_n191));
  nand02aa1d04x5               g096(.a(new_n191), .b(new_n173), .o1(new_n192));
  nano22aa1d15x5               g097(.a(new_n192), .b(new_n132), .c(new_n149), .out0(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n194));
  oai012aa1n02x5               g099(.a(new_n186), .b(new_n185), .c(new_n178), .o1(new_n195));
  oai013aa1n02x5               g100(.a(new_n195), .b(new_n183), .c(new_n176), .d(new_n187), .o1(new_n196));
  aoib12aa1n12x5               g101(.a(new_n196), .b(new_n167), .c(new_n192), .out0(new_n197));
  xorc02aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n194), .c(new_n197), .out0(\s[17] ));
  inv000aa1d42x5               g104(.a(\a[17] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(\b[16] ), .b(new_n200), .out0(new_n201));
  oabi12aa1n06x5               g106(.a(new_n196), .b(new_n150), .c(new_n192), .out0(new_n202));
  aoai13aa1n03x5               g107(.a(new_n198), .b(new_n202), .c(new_n165), .d(new_n193), .o1(new_n203));
  xorc02aa1n02x5               g108(.a(\a[18] ), .b(\b[17] ), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n203), .c(new_n201), .out0(\s[18] ));
  inv040aa1d32x5               g110(.a(\a[18] ), .o1(new_n206));
  xroi22aa1d04x5               g111(.a(new_n200), .b(\b[16] ), .c(new_n206), .d(\b[17] ), .out0(new_n207));
  aoai13aa1n03x5               g112(.a(new_n207), .b(new_n202), .c(new_n165), .d(new_n193), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n207), .o1(new_n209));
  nor042aa1n06x5               g114(.a(\b[16] ), .b(\a[17] ), .o1(new_n210));
  inv000aa1d42x5               g115(.a(\b[17] ), .o1(new_n211));
  nand42aa1n16x5               g116(.a(new_n211), .b(new_n206), .o1(new_n212));
  nand22aa1n12x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  aob012aa1d18x5               g118(.a(new_n212), .b(new_n210), .c(new_n213), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n209), .c(new_n194), .d(new_n197), .o1(new_n216));
  nor042aa1n09x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nand02aa1n06x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  norb02aa1n12x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n212), .o1(new_n220));
  aoi112aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n213), .d(new_n210), .o1(new_n221));
  aoi022aa1n02x5               g126(.a(new_n216), .b(new_n219), .c(new_n208), .d(new_n221), .o1(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nand02aa1d10x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  norb02aa1n12x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n217), .c(new_n216), .d(new_n218), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n219), .o1(new_n229));
  norb03aa1n02x5               g134(.a(new_n225), .b(new_n217), .c(new_n224), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n229), .c(new_n208), .d(new_n215), .o1(new_n231));
  nanp02aa1n03x5               g136(.a(new_n228), .b(new_n231), .o1(\s[20] ));
  nano23aa1n03x7               g137(.a(new_n217), .b(new_n224), .c(new_n225), .d(new_n218), .out0(new_n233));
  nand23aa1n03x5               g138(.a(new_n233), .b(new_n198), .c(new_n204), .o1(new_n234));
  oaih12aa1n06x5               g139(.a(new_n225), .b(new_n224), .c(new_n217), .o1(new_n235));
  aobi12aa1n06x5               g140(.a(new_n235), .b(new_n233), .c(new_n214), .out0(new_n236));
  aoai13aa1n04x5               g141(.a(new_n236), .b(new_n234), .c(new_n194), .d(new_n197), .o1(new_n237));
  nor042aa1d18x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  nand22aa1n12x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  nanp02aa1n06x5               g145(.a(new_n194), .b(new_n197), .o1(new_n241));
  inv000aa1n02x5               g146(.a(new_n236), .o1(new_n242));
  aoi113aa1n03x5               g147(.a(new_n242), .b(new_n240), .c(new_n241), .d(new_n207), .e(new_n233), .o1(new_n243));
  aoi012aa1n02x7               g148(.a(new_n243), .b(new_n237), .c(new_n240), .o1(\s[21] ));
  norp02aa1n24x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nand22aa1n12x5               g150(.a(\b[21] ), .b(\a[22] ), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n238), .c(new_n237), .d(new_n240), .o1(new_n249));
  norb03aa1n02x5               g154(.a(new_n246), .b(new_n238), .c(new_n245), .out0(new_n250));
  aob012aa1n03x5               g155(.a(new_n250), .b(new_n237), .c(new_n240), .out0(new_n251));
  nanp02aa1n03x5               g156(.a(new_n249), .b(new_n251), .o1(\s[22] ));
  nano23aa1n06x5               g157(.a(new_n238), .b(new_n245), .c(new_n246), .d(new_n239), .out0(new_n253));
  nanp03aa1n02x5               g158(.a(new_n207), .b(new_n233), .c(new_n253), .o1(new_n254));
  nanb02aa1n02x5               g159(.a(new_n254), .b(new_n241), .out0(new_n255));
  ao0012aa1n03x7               g160(.a(new_n245), .b(new_n238), .c(new_n246), .o(new_n256));
  aoi012aa1n02x5               g161(.a(new_n256), .b(new_n242), .c(new_n253), .o1(new_n257));
  aoai13aa1n04x5               g162(.a(new_n257), .b(new_n254), .c(new_n194), .d(new_n197), .o1(new_n258));
  nor022aa1n16x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  nanp02aa1n06x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  norb02aa1n02x5               g165(.a(new_n260), .b(new_n259), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n261), .b(new_n245), .c(new_n246), .d(new_n238), .o1(new_n262));
  aobi12aa1n02x5               g167(.a(new_n262), .b(new_n242), .c(new_n253), .out0(new_n263));
  aoi022aa1n02x5               g168(.a(new_n263), .b(new_n255), .c(new_n258), .d(new_n261), .o1(\s[23] ));
  nor022aa1n08x5               g169(.a(\b[23] ), .b(\a[24] ), .o1(new_n265));
  nand42aa1n16x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  nanb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(new_n267));
  aoai13aa1n03x5               g172(.a(new_n267), .b(new_n259), .c(new_n258), .d(new_n261), .o1(new_n268));
  nona22aa1n09x5               g173(.a(new_n266), .b(new_n265), .c(new_n259), .out0(new_n269));
  aoi012aa1n03x5               g174(.a(new_n269), .b(new_n258), .c(new_n261), .o1(new_n270));
  nanb02aa1n03x5               g175(.a(new_n270), .b(new_n268), .out0(\s[24] ));
  nano23aa1n09x5               g176(.a(new_n259), .b(new_n265), .c(new_n266), .d(new_n260), .out0(new_n272));
  nand22aa1n03x5               g177(.a(new_n272), .b(new_n253), .o1(new_n273));
  nano32aa1n02x4               g178(.a(new_n273), .b(new_n233), .c(new_n204), .d(new_n198), .out0(new_n274));
  inv000aa1n02x5               g179(.a(new_n274), .o1(new_n275));
  nand23aa1n06x5               g180(.a(new_n214), .b(new_n219), .c(new_n226), .o1(new_n276));
  aoi022aa1n06x5               g181(.a(new_n272), .b(new_n256), .c(new_n266), .d(new_n269), .o1(new_n277));
  aoai13aa1n12x5               g182(.a(new_n277), .b(new_n273), .c(new_n276), .d(new_n235), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  aoai13aa1n04x5               g184(.a(new_n279), .b(new_n275), .c(new_n194), .d(new_n197), .o1(new_n280));
  tech160nm_fixorc02aa1n04x5   g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  aoi112aa1n03x4               g186(.a(new_n281), .b(new_n278), .c(new_n241), .d(new_n274), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(\s[25] ));
  norp02aa1n02x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  aoi012aa1n03x5               g189(.a(new_n284), .b(new_n280), .c(new_n281), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  norb02aa1n02x5               g191(.a(new_n286), .b(new_n284), .out0(new_n287));
  aob012aa1n03x5               g192(.a(new_n287), .b(new_n280), .c(new_n281), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n285), .c(new_n286), .o1(\s[26] ));
  and002aa1n02x5               g194(.a(new_n286), .b(new_n281), .o(new_n290));
  norb03aa1n03x5               g195(.a(new_n290), .b(new_n234), .c(new_n273), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n202), .c(new_n165), .d(new_n193), .o1(new_n292));
  inv000aa1n02x5               g197(.a(new_n291), .o1(new_n293));
  oaoi03aa1n02x5               g198(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .o1(new_n294));
  aoi012aa1n12x5               g199(.a(new_n294), .b(new_n278), .c(new_n290), .o1(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n293), .c(new_n194), .d(new_n197), .o1(new_n296));
  xorc02aa1n12x5               g201(.a(\a[27] ), .b(\b[26] ), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  oai012aa1n02x5               g203(.a(new_n298), .b(\b[25] ), .c(\a[26] ), .o1(new_n299));
  aoi122aa1n02x5               g204(.a(new_n299), .b(new_n284), .c(new_n286), .d(new_n278), .e(new_n290), .o1(new_n300));
  aoi022aa1n02x5               g205(.a(new_n296), .b(new_n297), .c(new_n292), .d(new_n300), .o1(\s[27] ));
  norp02aa1n02x5               g206(.a(\b[26] ), .b(\a[27] ), .o1(new_n302));
  xnrc02aa1n12x5               g207(.a(\b[27] ), .b(\a[28] ), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n302), .c(new_n296), .d(new_n297), .o1(new_n304));
  norp02aa1n02x5               g209(.a(new_n303), .b(new_n302), .o1(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n298), .c(new_n292), .d(new_n295), .o1(new_n306));
  nanp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[28] ));
  norb02aa1n12x5               g212(.a(new_n297), .b(new_n303), .out0(new_n308));
  tech160nm_fiaoi012aa1n05x5   g213(.a(new_n305), .b(\a[28] ), .c(\b[27] ), .o1(new_n309));
  xorc02aa1n12x5               g214(.a(\a[29] ), .b(\b[28] ), .out0(new_n310));
  inv000aa1d42x5               g215(.a(new_n310), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n309), .c(new_n296), .d(new_n308), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n308), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n309), .b(new_n311), .o1(new_n314));
  aoai13aa1n02x5               g219(.a(new_n314), .b(new_n313), .c(new_n292), .d(new_n295), .o1(new_n315));
  nanp02aa1n03x5               g220(.a(new_n312), .b(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g222(.a(new_n303), .b(new_n297), .c(new_n310), .out0(new_n318));
  nanp02aa1n03x5               g223(.a(new_n296), .b(new_n318), .o1(new_n319));
  norp02aa1n02x5               g224(.a(\b[28] ), .b(\a[29] ), .o1(new_n320));
  aoi012aa1n02x5               g225(.a(new_n320), .b(new_n309), .c(new_n310), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  inv000aa1n02x5               g227(.a(new_n318), .o1(new_n323));
  oai012aa1n02x5               g228(.a(new_n322), .b(\b[28] ), .c(\a[29] ), .o1(new_n324));
  aoi012aa1n02x5               g229(.a(new_n324), .b(new_n309), .c(new_n310), .o1(new_n325));
  aoai13aa1n02x5               g230(.a(new_n325), .b(new_n323), .c(new_n292), .d(new_n295), .o1(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n322), .c(new_n319), .d(new_n321), .o1(\s[30] ));
  nano23aa1n06x5               g232(.a(new_n311), .b(new_n303), .c(new_n322), .d(new_n297), .out0(new_n328));
  aoi012aa1n02x5               g233(.a(new_n325), .b(\a[30] ), .c(\b[29] ), .o1(new_n329));
  xnrc02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n329), .c(new_n296), .d(new_n328), .o1(new_n331));
  inv000aa1n02x5               g236(.a(new_n328), .o1(new_n332));
  norp02aa1n02x5               g237(.a(new_n329), .b(new_n330), .o1(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n332), .c(new_n292), .d(new_n295), .o1(new_n334));
  nanp02aa1n03x5               g239(.a(new_n331), .b(new_n334), .o1(\s[31] ));
  aoi112aa1n02x5               g240(.a(new_n157), .b(new_n99), .c(new_n100), .d(new_n101), .o1(new_n336));
  aoib12aa1n02x5               g241(.a(new_n336), .b(new_n157), .c(new_n102), .out0(\s[3] ));
  inv000aa1d42x5               g242(.a(new_n105), .o1(new_n338));
  aoai13aa1n02x5               g243(.a(new_n157), .b(new_n99), .c(new_n101), .d(new_n100), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n156), .b(new_n339), .c(new_n338), .out0(\s[4] ));
  aoi112aa1n02x5               g245(.a(new_n159), .b(new_n103), .c(new_n104), .d(new_n105), .o1(new_n341));
  aoi022aa1n02x5               g246(.a(new_n109), .b(new_n159), .c(new_n158), .d(new_n341), .o1(\s[5] ));
  aoai13aa1n02x5               g247(.a(new_n160), .b(new_n110), .c(new_n109), .d(new_n111), .o1(new_n343));
  nona22aa1n02x4               g248(.a(new_n113), .b(new_n112), .c(new_n110), .out0(new_n344));
  aoai13aa1n02x5               g249(.a(new_n343), .b(new_n344), .c(new_n159), .d(new_n109), .o1(\s[6] ));
  nanb02aa1n02x5               g250(.a(new_n114), .b(new_n109), .out0(new_n346));
  aoi012aa1n02x5               g251(.a(new_n116), .b(new_n346), .c(new_n118), .o1(new_n347));
  aoi112aa1n02x5               g252(.a(new_n161), .b(new_n112), .c(new_n113), .d(new_n110), .o1(new_n348));
  aoi012aa1n02x5               g253(.a(new_n347), .b(new_n346), .c(new_n348), .o1(\s[7] ));
  norp02aa1n02x5               g254(.a(new_n115), .b(new_n121), .o1(new_n350));
  oai012aa1n02x5               g255(.a(new_n115), .b(new_n347), .c(new_n121), .o1(new_n351));
  oaib12aa1n02x5               g256(.a(new_n351), .b(new_n347), .c(new_n350), .out0(\s[8] ));
  xorb03aa1n02x5               g257(.a(new_n165), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


