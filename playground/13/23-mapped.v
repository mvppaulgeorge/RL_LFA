// Benchmark "adder" written by ABC on Wed Jul 17 18:48:18 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n179, new_n181, new_n182, new_n183, new_n184, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n219, new_n220, new_n221, new_n222, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n240,
    new_n241, new_n242, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n261, new_n262, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n347, new_n348, new_n351, new_n353, new_n354, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n06x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  inv000aa1d42x5               g004(.a(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[8] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand02aa1n04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nand02aa1n04x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  aoi012aa1d18x5               g010(.a(new_n103), .b(new_n104), .c(new_n105), .o1(new_n106));
  nor002aa1n12x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nand22aa1n04x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  norp02aa1n12x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nand42aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nona23aa1n09x5               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  inv000aa1d42x5               g016(.a(\a[3] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[2] ), .o1(new_n113));
  aoai13aa1n06x5               g018(.a(new_n108), .b(new_n107), .c(new_n112), .d(new_n113), .o1(new_n114));
  oaih12aa1n06x5               g019(.a(new_n114), .b(new_n111), .c(new_n106), .o1(new_n115));
  nor042aa1n06x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand02aa1n04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nanb02aa1n02x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  norp02aa1n04x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nanp02aa1n06x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  norp02aa1n04x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nona23aa1n03x5               g027(.a(new_n122), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n123));
  nor042aa1d18x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  nand02aa1n08x5               g029(.a(\b[6] ), .b(\a[7] ), .o1(new_n125));
  nanb02aa1n06x5               g030(.a(new_n124), .b(new_n125), .out0(new_n126));
  nor043aa1n03x5               g031(.a(new_n123), .b(new_n126), .c(new_n118), .o1(new_n127));
  aoi112aa1n02x5               g032(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n128));
  norb02aa1n06x5               g033(.a(new_n117), .b(new_n116), .out0(new_n129));
  oai022aa1n02x5               g034(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n130));
  nano22aa1n03x7               g035(.a(new_n124), .b(new_n120), .c(new_n125), .out0(new_n131));
  nand43aa1n02x5               g036(.a(new_n131), .b(new_n129), .c(new_n130), .o1(new_n132));
  nona22aa1n03x5               g037(.a(new_n132), .b(new_n128), .c(new_n116), .out0(new_n133));
  xorc02aa1n03x5               g038(.a(\a[9] ), .b(\b[8] ), .out0(new_n134));
  aoai13aa1n06x5               g039(.a(new_n134), .b(new_n133), .c(new_n115), .d(new_n127), .o1(new_n135));
  xobna2aa1n03x5               g040(.a(new_n99), .b(new_n135), .c(new_n102), .out0(\s[10] ));
  aoai13aa1n04x5               g041(.a(new_n98), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n137));
  oai012aa1n04x7               g042(.a(new_n137), .b(new_n135), .c(new_n99), .o1(new_n138));
  xorb03aa1n02x5               g043(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand02aa1n03x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nor042aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n06x4               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n140), .c(new_n138), .d(new_n141), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(new_n140), .b(new_n144), .c(new_n138), .d(new_n141), .o1(new_n146));
  norb02aa1n03x4               g051(.a(new_n145), .b(new_n146), .out0(\s[12] ));
  inv000aa1n02x5               g052(.a(new_n98), .o1(new_n148));
  nona32aa1n03x5               g053(.a(new_n134), .b(new_n148), .c(new_n140), .d(new_n97), .out0(new_n149));
  nano22aa1n03x7               g054(.a(new_n142), .b(new_n141), .c(new_n143), .out0(new_n150));
  norb02aa1n02x7               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n133), .c(new_n115), .d(new_n127), .o1(new_n152));
  inv000aa1d42x5               g057(.a(\b[11] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(\a[12] ), .b(new_n153), .out0(new_n154));
  aoi112aa1n09x5               g059(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n155));
  inv000aa1n02x5               g060(.a(new_n155), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n140), .b(new_n141), .out0(new_n157));
  nanp02aa1n02x5               g062(.a(new_n154), .b(new_n143), .o1(new_n158));
  nor043aa1n02x5               g063(.a(new_n137), .b(new_n157), .c(new_n158), .o1(new_n159));
  nano22aa1n03x7               g064(.a(new_n159), .b(new_n154), .c(new_n156), .out0(new_n160));
  xorc02aa1n12x5               g065(.a(\a[13] ), .b(\b[12] ), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n152), .c(new_n160), .out0(\s[13] ));
  inv000aa1d42x5               g067(.a(\a[13] ), .o1(new_n163));
  nanb02aa1d24x5               g068(.a(\b[12] ), .b(new_n163), .out0(new_n164));
  inv000aa1n02x5               g069(.a(new_n106), .o1(new_n165));
  nano23aa1n03x7               g070(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n166));
  aobi12aa1n06x5               g071(.a(new_n114), .b(new_n166), .c(new_n165), .out0(new_n167));
  nano23aa1n02x5               g072(.a(new_n119), .b(new_n121), .c(new_n122), .d(new_n120), .out0(new_n168));
  nona22aa1n02x4               g073(.a(new_n168), .b(new_n126), .c(new_n118), .out0(new_n169));
  aoi113aa1n02x5               g074(.a(new_n128), .b(new_n116), .c(new_n131), .d(new_n130), .e(new_n117), .o1(new_n170));
  tech160nm_fioai012aa1n03p5x5 g075(.a(new_n170), .b(new_n167), .c(new_n169), .o1(new_n171));
  oaoi03aa1n02x5               g076(.a(\a[10] ), .b(\b[9] ), .c(new_n102), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n141), .b(new_n140), .out0(new_n173));
  nanp03aa1n03x5               g078(.a(new_n172), .b(new_n173), .c(new_n144), .o1(new_n174));
  nona22aa1n09x5               g079(.a(new_n174), .b(new_n155), .c(new_n142), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n161), .b(new_n175), .c(new_n171), .d(new_n151), .o1(new_n176));
  nor042aa1n03x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  and002aa1n12x5               g082(.a(\b[13] ), .b(\a[14] ), .o(new_n178));
  nor042aa1n06x5               g083(.a(new_n178), .b(new_n177), .o1(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n176), .c(new_n164), .out0(\s[14] ));
  nanp02aa1n06x5               g085(.a(new_n161), .b(new_n179), .o1(new_n181));
  oaoi03aa1n12x5               g086(.a(\a[14] ), .b(\b[13] ), .c(new_n164), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n183), .b(new_n181), .c(new_n152), .d(new_n160), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n12x5               g090(.a(\b[14] ), .b(\a[15] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .o1(new_n187));
  nor042aa1n04x5               g092(.a(\b[15] ), .b(\a[16] ), .o1(new_n188));
  and002aa1n12x5               g093(.a(\b[15] ), .b(\a[16] ), .o(new_n189));
  norp02aa1n12x5               g094(.a(new_n189), .b(new_n188), .o1(new_n190));
  aoai13aa1n03x5               g095(.a(new_n190), .b(new_n186), .c(new_n184), .d(new_n187), .o1(new_n191));
  aoi112aa1n03x5               g096(.a(new_n186), .b(new_n190), .c(new_n184), .d(new_n187), .o1(new_n192));
  norb02aa1n03x4               g097(.a(new_n191), .b(new_n192), .out0(\s[16] ));
  nona32aa1n09x5               g098(.a(new_n161), .b(new_n186), .c(new_n178), .d(new_n177), .out0(new_n194));
  aoi112aa1n03x5               g099(.a(new_n189), .b(new_n188), .c(\a[15] ), .d(\b[14] ), .o1(new_n195));
  nano23aa1n06x5               g100(.a(new_n194), .b(new_n149), .c(new_n150), .d(new_n195), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n133), .c(new_n115), .d(new_n127), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n186), .o1(new_n198));
  nano22aa1n03x7               g103(.a(new_n181), .b(new_n195), .c(new_n198), .out0(new_n199));
  aoi112aa1n02x5               g104(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n200));
  norb02aa1n03x4               g105(.a(new_n187), .b(new_n186), .out0(new_n201));
  nanp03aa1n03x5               g106(.a(new_n182), .b(new_n201), .c(new_n190), .o1(new_n202));
  nona22aa1n03x5               g107(.a(new_n202), .b(new_n200), .c(new_n188), .out0(new_n203));
  aoi012aa1n12x5               g108(.a(new_n203), .b(new_n175), .c(new_n199), .o1(new_n204));
  xorc02aa1n12x5               g109(.a(\a[17] ), .b(\b[16] ), .out0(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n197), .c(new_n204), .out0(\s[17] ));
  inv040aa1d28x5               g111(.a(\a[17] ), .o1(new_n207));
  inv040aa1d32x5               g112(.a(\b[16] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(new_n208), .b(new_n207), .o1(new_n209));
  inv000aa1n02x5               g114(.a(new_n179), .o1(new_n210));
  nona23aa1n03x5               g115(.a(new_n195), .b(new_n161), .c(new_n210), .d(new_n186), .out0(new_n211));
  aoi113aa1n03x5               g116(.a(new_n200), .b(new_n188), .c(new_n182), .d(new_n190), .e(new_n201), .o1(new_n212));
  oai012aa1n03x5               g117(.a(new_n212), .b(new_n211), .c(new_n160), .o1(new_n213));
  aoai13aa1n02x5               g118(.a(new_n205), .b(new_n213), .c(new_n171), .d(new_n196), .o1(new_n214));
  nor002aa1d32x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  nand42aa1d28x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  nanb02aa1n12x5               g121(.a(new_n215), .b(new_n216), .out0(new_n217));
  xobna2aa1n03x5               g122(.a(new_n217), .b(new_n214), .c(new_n209), .out0(\s[18] ));
  norb02aa1d21x5               g123(.a(new_n205), .b(new_n217), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n12x5               g125(.a(new_n216), .b(new_n215), .c(new_n207), .d(new_n208), .o1(new_n221));
  aoai13aa1n02x7               g126(.a(new_n221), .b(new_n220), .c(new_n197), .d(new_n204), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g128(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g129(.a(\a[19] ), .o1(new_n225));
  inv040aa1d32x5               g130(.a(\b[18] ), .o1(new_n226));
  nand02aa1n16x5               g131(.a(new_n226), .b(new_n225), .o1(new_n227));
  tech160nm_fiaoi012aa1n05x5   g132(.a(new_n133), .b(new_n115), .c(new_n127), .o1(new_n228));
  nand02aa1n02x5               g133(.a(new_n151), .b(new_n199), .o1(new_n229));
  oai012aa1n18x5               g134(.a(new_n204), .b(new_n228), .c(new_n229), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n221), .o1(new_n231));
  nand02aa1n08x5               g136(.a(\b[18] ), .b(\a[19] ), .o1(new_n232));
  nanp02aa1n12x5               g137(.a(new_n227), .b(new_n232), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n231), .c(new_n230), .d(new_n219), .o1(new_n235));
  nor002aa1d32x5               g140(.a(\b[19] ), .b(\a[20] ), .o1(new_n236));
  nand02aa1n06x5               g141(.a(\b[19] ), .b(\a[20] ), .o1(new_n237));
  nanb02aa1d24x5               g142(.a(new_n236), .b(new_n237), .out0(new_n238));
  tech160nm_fiaoi012aa1n02p5x5 g143(.a(new_n238), .b(new_n235), .c(new_n227), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n227), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n238), .o1(new_n241));
  aoi112aa1n03x4               g146(.a(new_n240), .b(new_n241), .c(new_n222), .d(new_n232), .o1(new_n242));
  norp02aa1n03x5               g147(.a(new_n239), .b(new_n242), .o1(\s[20] ));
  nona23aa1d16x5               g148(.a(new_n219), .b(new_n232), .c(new_n238), .d(new_n240), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n236), .o1(new_n245));
  aoi112aa1n06x5               g150(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n246));
  inv000aa1n02x5               g151(.a(new_n246), .o1(new_n247));
  nor043aa1d12x5               g152(.a(new_n221), .b(new_n238), .c(new_n233), .o1(new_n248));
  nano22aa1d15x5               g153(.a(new_n248), .b(new_n245), .c(new_n247), .out0(new_n249));
  aoai13aa1n04x5               g154(.a(new_n249), .b(new_n244), .c(new_n197), .d(new_n204), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g156(.a(\b[20] ), .b(\a[21] ), .o1(new_n252));
  inv040aa1n08x5               g157(.a(new_n252), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n244), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n249), .o1(new_n255));
  tech160nm_finand02aa1n05x5   g160(.a(\b[20] ), .b(\a[21] ), .o1(new_n256));
  norb02aa1n06x5               g161(.a(new_n256), .b(new_n252), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n255), .c(new_n230), .d(new_n254), .o1(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[21] ), .b(\a[22] ), .out0(new_n259));
  aoi012aa1n03x5               g164(.a(new_n259), .b(new_n258), .c(new_n253), .o1(new_n260));
  tech160nm_fixorc02aa1n05x5   g165(.a(\a[22] ), .b(\b[21] ), .out0(new_n261));
  aoi112aa1n03x4               g166(.a(new_n252), .b(new_n261), .c(new_n250), .d(new_n257), .o1(new_n262));
  nor002aa1n02x5               g167(.a(new_n260), .b(new_n262), .o1(\s[22] ));
  nanp02aa1n03x5               g168(.a(new_n261), .b(new_n257), .o1(new_n264));
  nor042aa1n02x5               g169(.a(new_n244), .b(new_n264), .o1(new_n265));
  inv020aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  oaoi03aa1n12x5               g171(.a(\a[22] ), .b(\b[21] ), .c(new_n253), .o1(new_n267));
  oab012aa1n06x5               g172(.a(new_n267), .b(new_n249), .c(new_n264), .out0(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n266), .c(new_n197), .d(new_n204), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n20x5               g175(.a(\b[22] ), .b(\a[23] ), .o1(new_n271));
  inv040aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  inv030aa1n02x5               g177(.a(new_n268), .o1(new_n273));
  nand02aa1d08x5               g178(.a(\b[22] ), .b(\a[23] ), .o1(new_n274));
  norb02aa1n03x5               g179(.a(new_n274), .b(new_n271), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n273), .c(new_n230), .d(new_n265), .o1(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[23] ), .b(\a[24] ), .out0(new_n277));
  aoi012aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n272), .o1(new_n278));
  xorc02aa1n12x5               g183(.a(\a[24] ), .b(\b[23] ), .out0(new_n279));
  aoi112aa1n03x4               g184(.a(new_n271), .b(new_n279), .c(new_n269), .d(new_n274), .o1(new_n280));
  nor002aa1n02x5               g185(.a(new_n278), .b(new_n280), .o1(\s[24] ));
  nano32aa1n02x4               g186(.a(new_n259), .b(new_n253), .c(new_n272), .d(new_n256), .out0(new_n282));
  nand23aa1n03x5               g187(.a(new_n282), .b(new_n274), .c(new_n279), .o1(new_n283));
  nor042aa1n02x5               g188(.a(new_n244), .b(new_n283), .o1(new_n284));
  inv000aa1n02x5               g189(.a(new_n284), .o1(new_n285));
  norp02aa1n02x5               g190(.a(\b[23] ), .b(\a[24] ), .o1(new_n286));
  aoi112aa1n02x5               g191(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n287));
  aoi113aa1n06x5               g192(.a(new_n287), .b(new_n286), .c(new_n267), .d(new_n279), .e(new_n275), .o1(new_n288));
  oai012aa1d24x5               g193(.a(new_n288), .b(new_n249), .c(new_n283), .o1(new_n289));
  inv040aa1n06x5               g194(.a(new_n289), .o1(new_n290));
  aoai13aa1n04x5               g195(.a(new_n290), .b(new_n285), .c(new_n197), .d(new_n204), .o1(new_n291));
  xorb03aa1n02x5               g196(.a(new_n291), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g197(.a(\b[24] ), .b(\a[25] ), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  xorc02aa1n12x5               g199(.a(\a[25] ), .b(\b[24] ), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n289), .c(new_n230), .d(new_n284), .o1(new_n296));
  xorc02aa1n12x5               g201(.a(\a[26] ), .b(\b[25] ), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  aoi012aa1n02x7               g203(.a(new_n298), .b(new_n296), .c(new_n294), .o1(new_n299));
  aoi112aa1n02x5               g204(.a(new_n293), .b(new_n297), .c(new_n291), .d(new_n295), .o1(new_n300));
  nor002aa1n02x5               g205(.a(new_n299), .b(new_n300), .o1(\s[26] ));
  nano32aa1n03x7               g206(.a(new_n264), .b(new_n279), .c(new_n272), .d(new_n274), .out0(new_n302));
  and002aa1n24x5               g207(.a(new_n297), .b(new_n295), .o(new_n303));
  nano22aa1d15x5               g208(.a(new_n244), .b(new_n302), .c(new_n303), .out0(new_n304));
  aoai13aa1n06x5               g209(.a(new_n304), .b(new_n213), .c(new_n171), .d(new_n196), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[26] ), .b(\b[25] ), .c(new_n294), .carry(new_n306));
  aobi12aa1d24x5               g211(.a(new_n306), .b(new_n289), .c(new_n303), .out0(new_n307));
  xorc02aa1n12x5               g212(.a(\a[27] ), .b(\b[26] ), .out0(new_n308));
  xnbna2aa1n06x5               g213(.a(new_n308), .b(new_n305), .c(new_n307), .out0(\s[27] ));
  norp02aa1n02x5               g214(.a(\b[26] ), .b(\a[27] ), .o1(new_n310));
  inv040aa1n03x5               g215(.a(new_n310), .o1(new_n311));
  oai013aa1n03x5               g216(.a(new_n302), .b(new_n248), .c(new_n236), .d(new_n246), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n303), .o1(new_n313));
  aoai13aa1n04x5               g218(.a(new_n306), .b(new_n313), .c(new_n312), .d(new_n288), .o1(new_n314));
  aoai13aa1n02x7               g219(.a(new_n308), .b(new_n314), .c(new_n230), .d(new_n304), .o1(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[27] ), .b(\a[28] ), .out0(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n315), .c(new_n311), .o1(new_n317));
  aobi12aa1n06x5               g222(.a(new_n308), .b(new_n305), .c(new_n307), .out0(new_n318));
  nano22aa1n03x5               g223(.a(new_n318), .b(new_n311), .c(new_n316), .out0(new_n319));
  nor002aa1n02x5               g224(.a(new_n317), .b(new_n319), .o1(\s[28] ));
  norb02aa1n02x5               g225(.a(new_n308), .b(new_n316), .out0(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n314), .c(new_n230), .d(new_n304), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[28] ), .b(\b[27] ), .c(new_n311), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(\b[28] ), .b(\a[29] ), .out0(new_n324));
  tech160nm_fiaoi012aa1n02p5x5 g229(.a(new_n324), .b(new_n322), .c(new_n323), .o1(new_n325));
  aobi12aa1n06x5               g230(.a(new_n321), .b(new_n305), .c(new_n307), .out0(new_n326));
  nano22aa1n03x5               g231(.a(new_n326), .b(new_n323), .c(new_n324), .out0(new_n327));
  nor002aa1n02x5               g232(.a(new_n325), .b(new_n327), .o1(\s[29] ));
  xorb03aa1n02x5               g233(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g234(.a(new_n308), .b(new_n324), .c(new_n316), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n314), .c(new_n230), .d(new_n304), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[29] ), .b(\b[28] ), .c(new_n323), .carry(new_n332));
  xnrc02aa1n02x5               g237(.a(\b[29] ), .b(\a[30] ), .out0(new_n333));
  tech160nm_fiaoi012aa1n02p5x5 g238(.a(new_n333), .b(new_n331), .c(new_n332), .o1(new_n334));
  aobi12aa1n06x5               g239(.a(new_n330), .b(new_n305), .c(new_n307), .out0(new_n335));
  nano22aa1n03x5               g240(.a(new_n335), .b(new_n332), .c(new_n333), .out0(new_n336));
  norp02aa1n03x5               g241(.a(new_n334), .b(new_n336), .o1(\s[30] ));
  norb02aa1n02x5               g242(.a(new_n330), .b(new_n333), .out0(new_n338));
  aobi12aa1n06x5               g243(.a(new_n338), .b(new_n305), .c(new_n307), .out0(new_n339));
  oao003aa1n02x5               g244(.a(\a[30] ), .b(\b[29] ), .c(new_n332), .carry(new_n340));
  xnrc02aa1n02x5               g245(.a(\b[30] ), .b(\a[31] ), .out0(new_n341));
  nano22aa1n03x5               g246(.a(new_n339), .b(new_n340), .c(new_n341), .out0(new_n342));
  aoai13aa1n03x5               g247(.a(new_n338), .b(new_n314), .c(new_n230), .d(new_n304), .o1(new_n343));
  tech160nm_fiaoi012aa1n02p5x5 g248(.a(new_n341), .b(new_n343), .c(new_n340), .o1(new_n344));
  norp02aa1n03x5               g249(.a(new_n344), .b(new_n342), .o1(\s[31] ));
  xorb03aa1n02x5               g250(.a(new_n106), .b(\b[2] ), .c(new_n112), .out0(\s[3] ));
  nona22aa1n02x4               g251(.a(new_n110), .b(new_n106), .c(new_n109), .out0(new_n347));
  aboi22aa1n03x5               g252(.a(new_n107), .b(new_n108), .c(new_n112), .d(new_n113), .out0(new_n348));
  aboi22aa1n03x5               g253(.a(new_n107), .b(new_n115), .c(new_n347), .d(new_n348), .out0(\s[4] ));
  xorb03aa1n02x5               g254(.a(new_n115), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g255(.a(\a[5] ), .b(\b[4] ), .c(new_n167), .o1(new_n351));
  xorb03aa1n02x5               g256(.a(new_n351), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai012aa1n02x5               g257(.a(new_n131), .b(new_n351), .c(new_n119), .o1(new_n353));
  inv000aa1d42x5               g258(.a(new_n124), .o1(new_n354));
  aoi122aa1n02x5               g259(.a(new_n119), .b(new_n354), .c(new_n125), .d(new_n351), .e(new_n120), .o1(new_n355));
  norb02aa1n02x5               g260(.a(new_n353), .b(new_n355), .out0(\s[7] ));
  xnbna2aa1n03x5               g261(.a(new_n129), .b(new_n353), .c(new_n354), .out0(\s[8] ));
  xorb03aa1n02x5               g262(.a(new_n228), .b(\b[8] ), .c(new_n100), .out0(\s[9] ));
endmodule


