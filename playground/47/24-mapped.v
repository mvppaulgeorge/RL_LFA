// Benchmark "adder" written by ABC on Thu Jul 18 12:17:29 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n337,
    new_n338, new_n339, new_n341, new_n342, new_n344, new_n346, new_n347,
    new_n348, new_n349, new_n351, new_n352, new_n354, new_n355, new_n357,
    new_n358, new_n359;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  and002aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n12x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  nor002aa1n03x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norb02aa1n06x4               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  norp02aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n06x4               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nanp03aa1d12x5               g011(.a(new_n100), .b(new_n103), .c(new_n106), .o1(new_n107));
  tech160nm_fiaoi012aa1n05x5   g012(.a(new_n101), .b(new_n104), .c(new_n102), .o1(new_n108));
  nanp02aa1n06x5               g013(.a(new_n107), .b(new_n108), .o1(new_n109));
  nand02aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor042aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor042aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n06x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  oai022aa1n12x5               g019(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n115));
  nand22aa1n09x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nanb03aa1n02x5               g022(.a(new_n115), .b(new_n117), .c(new_n116), .out0(new_n118));
  norb02aa1n06x4               g023(.a(new_n114), .b(new_n118), .out0(new_n119));
  norb03aa1n09x5               g024(.a(new_n113), .b(new_n111), .c(new_n112), .out0(new_n120));
  nand23aa1n03x5               g025(.a(new_n115), .b(new_n110), .c(new_n116), .o1(new_n121));
  tech160nm_fiaoi012aa1n05x5   g026(.a(new_n112), .b(new_n111), .c(new_n110), .o1(new_n122));
  oaib12aa1n09x5               g027(.a(new_n122), .b(new_n121), .c(new_n120), .out0(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n109), .d(new_n119), .o1(new_n125));
  tech160nm_fixorc02aa1n03p5x5 g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  oa0022aa1n03x5               g032(.a(\b[9] ), .b(\a[10] ), .c(\b[8] ), .d(\a[9] ), .o(new_n128));
  nand42aa1n02x5               g033(.a(new_n125), .b(new_n128), .o1(new_n129));
  nor042aa1n06x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand42aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  aob012aa1n02x5               g036(.a(new_n131), .b(\b[9] ), .c(\a[10] ), .out0(new_n132));
  nona22aa1n02x4               g037(.a(new_n129), .b(new_n130), .c(new_n132), .out0(new_n133));
  norb02aa1n02x5               g038(.a(new_n131), .b(new_n130), .out0(new_n134));
  aoi022aa1n02x5               g039(.a(new_n125), .b(new_n128), .c(\b[9] ), .d(\a[10] ), .o1(new_n135));
  oa0012aa1n02x5               g040(.a(new_n133), .b(new_n135), .c(new_n134), .o(\s[11] ));
  nor042aa1n03x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoib12aa1n02x5               g044(.a(new_n130), .b(new_n138), .c(new_n137), .out0(new_n140));
  tech160nm_fioai012aa1n03p5x5 g045(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .o1(new_n141));
  aoi022aa1n02x5               g046(.a(new_n141), .b(new_n139), .c(new_n133), .d(new_n140), .o1(\s[12] ));
  nano23aa1n06x5               g047(.a(new_n130), .b(new_n137), .c(new_n138), .d(new_n131), .out0(new_n143));
  and003aa1n02x5               g048(.a(new_n143), .b(new_n126), .c(new_n124), .o(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n123), .c(new_n109), .d(new_n119), .o1(new_n145));
  norb03aa1n03x5               g050(.a(new_n138), .b(new_n130), .c(new_n137), .out0(new_n146));
  nona22aa1n09x5               g051(.a(new_n146), .b(new_n128), .c(new_n132), .out0(new_n147));
  tech160nm_fiaoi012aa1n03p5x5 g052(.a(new_n137), .b(new_n130), .c(new_n138), .o1(new_n148));
  and002aa1n06x5               g053(.a(new_n147), .b(new_n148), .o(new_n149));
  nanp02aa1n02x5               g054(.a(new_n145), .b(new_n149), .o1(new_n150));
  xnrc02aa1n12x5               g055(.a(\b[12] ), .b(\a[13] ), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  and003aa1n02x5               g057(.a(new_n147), .b(new_n151), .c(new_n148), .o(new_n153));
  aoi022aa1n02x5               g058(.a(new_n150), .b(new_n152), .c(new_n145), .d(new_n153), .o1(\s[13] ));
  orn002aa1n03x5               g059(.a(\a[13] ), .b(\b[12] ), .o(new_n155));
  inv000aa1d42x5               g060(.a(new_n116), .o1(new_n156));
  nona23aa1n03x5               g061(.a(new_n114), .b(new_n117), .c(new_n156), .d(new_n115), .out0(new_n157));
  inv000aa1n02x5               g062(.a(new_n122), .o1(new_n158));
  aoib12aa1n06x5               g063(.a(new_n158), .b(new_n120), .c(new_n121), .out0(new_n159));
  aoai13aa1n04x5               g064(.a(new_n159), .b(new_n157), .c(new_n107), .d(new_n108), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n149), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n152), .b(new_n161), .c(new_n160), .d(new_n144), .o1(new_n162));
  tech160nm_fixnrc02aa1n04x5   g067(.a(\b[13] ), .b(\a[14] ), .out0(new_n163));
  xobna2aa1n03x5               g068(.a(new_n163), .b(new_n162), .c(new_n155), .out0(\s[14] ));
  norp02aa1n02x5               g069(.a(new_n163), .b(new_n151), .o1(new_n165));
  aoai13aa1n03x5               g070(.a(new_n165), .b(new_n161), .c(new_n160), .d(new_n144), .o1(new_n166));
  oaoi03aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n155), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  nor002aa1d32x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand22aa1n09x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanb02aa1d24x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n166), .c(new_n168), .out0(\s[15] ));
  aoai13aa1n06x5               g078(.a(new_n172), .b(new_n167), .c(new_n150), .d(new_n165), .o1(new_n174));
  xorc02aa1n02x5               g079(.a(\a[16] ), .b(\b[15] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(\a[16] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(\b[15] ), .o1(new_n177));
  nand02aa1n02x5               g082(.a(new_n177), .b(new_n176), .o1(new_n178));
  nanp02aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  aoi012aa1n02x5               g084(.a(new_n169), .b(new_n178), .c(new_n179), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n169), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n181), .b(new_n171), .c(new_n166), .d(new_n168), .o1(new_n182));
  aoi022aa1n03x5               g087(.a(new_n182), .b(new_n175), .c(new_n174), .d(new_n180), .o1(\s[16] ));
  nano22aa1n03x7               g088(.a(new_n171), .b(new_n178), .c(new_n179), .out0(new_n184));
  nona22aa1n09x5               g089(.a(new_n184), .b(new_n163), .c(new_n151), .out0(new_n185));
  nano32aa1n09x5               g090(.a(new_n185), .b(new_n143), .c(new_n126), .d(new_n124), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n123), .c(new_n109), .d(new_n119), .o1(new_n187));
  nanp03aa1n02x5               g092(.a(new_n178), .b(new_n170), .c(new_n179), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\a[14] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[13] ), .o1(new_n190));
  oai022aa1n02x7               g095(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n191));
  oai122aa1n02x7               g096(.a(new_n191), .b(\a[15] ), .c(\b[14] ), .d(new_n189), .e(new_n190), .o1(new_n192));
  oaoi03aa1n02x5               g097(.a(new_n176), .b(new_n177), .c(new_n169), .o1(new_n193));
  oa0012aa1n02x5               g098(.a(new_n193), .b(new_n192), .c(new_n188), .o(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n185), .c(new_n147), .d(new_n148), .o1(new_n195));
  nanb02aa1n12x5               g100(.a(new_n195), .b(new_n187), .out0(new_n196));
  xorc02aa1n12x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  aoi012aa1n02x5               g102(.a(new_n185), .b(new_n147), .c(new_n148), .o1(new_n198));
  orn002aa1n02x5               g103(.a(new_n192), .b(new_n188), .o(new_n199));
  nano23aa1n02x4               g104(.a(new_n198), .b(new_n197), .c(new_n199), .d(new_n193), .out0(new_n200));
  aoi022aa1n02x5               g105(.a(new_n196), .b(new_n197), .c(new_n187), .d(new_n200), .o1(\s[17] ));
  nor042aa1d18x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n02x5               g108(.a(new_n197), .b(new_n195), .c(new_n160), .d(new_n186), .o1(new_n204));
  nor042aa1n09x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand02aa1d16x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  norb02aa1n06x4               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n204), .c(new_n203), .out0(\s[18] ));
  and002aa1n02x5               g113(.a(new_n197), .b(new_n207), .o(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n195), .c(new_n160), .d(new_n186), .o1(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n203), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nor042aa1d18x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand02aa1d28x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norb02aa1n12x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n210), .c(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g122(.a(new_n215), .b(new_n211), .c(new_n196), .d(new_n209), .o1(new_n218));
  nor042aa1n09x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand02aa1d28x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  norb02aa1n06x4               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  inv000aa1d42x5               g126(.a(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[18] ), .o1(new_n223));
  aboi22aa1n03x5               g128(.a(new_n219), .b(new_n220), .c(new_n222), .d(new_n223), .out0(new_n224));
  inv040aa1n03x5               g129(.a(new_n213), .o1(new_n225));
  inv040aa1n02x5               g130(.a(new_n215), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n225), .b(new_n226), .c(new_n210), .d(new_n212), .o1(new_n227));
  aoi022aa1n03x5               g132(.a(new_n227), .b(new_n221), .c(new_n218), .d(new_n224), .o1(\s[20] ));
  nano32aa1n03x7               g133(.a(new_n226), .b(new_n197), .c(new_n221), .d(new_n207), .out0(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n195), .c(new_n160), .d(new_n186), .o1(new_n230));
  nanb03aa1d18x5               g135(.a(new_n219), .b(new_n220), .c(new_n214), .out0(new_n231));
  oai112aa1n06x5               g136(.a(new_n225), .b(new_n206), .c(new_n205), .d(new_n202), .o1(new_n232));
  aoi012aa1n12x5               g137(.a(new_n219), .b(new_n213), .c(new_n220), .o1(new_n233));
  oai012aa1n18x5               g138(.a(new_n233), .b(new_n232), .c(new_n231), .o1(new_n234));
  nor002aa1d32x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  nand02aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  norb02aa1d27x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n234), .c(new_n196), .d(new_n229), .o1(new_n238));
  nano22aa1n03x7               g143(.a(new_n219), .b(new_n214), .c(new_n220), .out0(new_n239));
  oaih12aa1n02x5               g144(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .o1(new_n240));
  oab012aa1n02x5               g145(.a(new_n240), .b(new_n202), .c(new_n205), .out0(new_n241));
  inv020aa1n03x5               g146(.a(new_n233), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(new_n242), .b(new_n237), .c(new_n241), .d(new_n239), .o1(new_n243));
  aobi12aa1n03x7               g148(.a(new_n238), .b(new_n243), .c(new_n230), .out0(\s[21] ));
  nor042aa1n04x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nanp02aa1n12x5               g150(.a(\b[21] ), .b(\a[22] ), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(new_n247));
  aoib12aa1n02x5               g152(.a(new_n235), .b(new_n246), .c(new_n245), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n234), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n235), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n237), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n250), .b(new_n251), .c(new_n230), .d(new_n249), .o1(new_n252));
  aoi022aa1n03x5               g157(.a(new_n252), .b(new_n247), .c(new_n238), .d(new_n248), .o1(\s[22] ));
  inv020aa1n03x5               g158(.a(new_n229), .o1(new_n254));
  nano22aa1n03x7               g159(.a(new_n254), .b(new_n237), .c(new_n247), .out0(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n195), .c(new_n160), .d(new_n186), .o1(new_n256));
  nano23aa1d12x5               g161(.a(new_n235), .b(new_n245), .c(new_n246), .d(new_n236), .out0(new_n257));
  aoi012aa1n12x5               g162(.a(new_n245), .b(new_n235), .c(new_n246), .o1(new_n258));
  inv020aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n234), .c(new_n257), .o1(new_n260));
  inv040aa1n03x5               g165(.a(new_n260), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n261), .c(new_n196), .d(new_n255), .o1(new_n263));
  aoi112aa1n02x5               g168(.a(new_n262), .b(new_n259), .c(new_n234), .d(new_n257), .o1(new_n264));
  aobi12aa1n02x7               g169(.a(new_n263), .b(new_n264), .c(new_n256), .out0(\s[23] ));
  tech160nm_fixorc02aa1n02p5x5 g170(.a(\a[24] ), .b(\b[23] ), .out0(new_n266));
  nor042aa1n06x5               g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  norp02aa1n02x5               g172(.a(new_n266), .b(new_n267), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n267), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n262), .o1(new_n270));
  aoai13aa1n03x5               g175(.a(new_n269), .b(new_n270), .c(new_n256), .d(new_n260), .o1(new_n271));
  aoi022aa1n02x7               g176(.a(new_n271), .b(new_n266), .c(new_n263), .d(new_n268), .o1(\s[24] ));
  and002aa1n12x5               g177(.a(new_n266), .b(new_n262), .o(new_n273));
  nano22aa1n12x5               g178(.a(new_n254), .b(new_n273), .c(new_n257), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n195), .c(new_n160), .d(new_n186), .o1(new_n275));
  aoai13aa1n04x5               g180(.a(new_n257), .b(new_n242), .c(new_n241), .d(new_n239), .o1(new_n276));
  inv000aa1n06x5               g181(.a(new_n273), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[24] ), .b(\b[23] ), .c(new_n269), .carry(new_n278));
  aoai13aa1n12x5               g183(.a(new_n278), .b(new_n277), .c(new_n276), .d(new_n258), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n279), .c(new_n196), .d(new_n274), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n273), .b(new_n259), .c(new_n234), .d(new_n257), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n280), .o1(new_n283));
  and003aa1n02x5               g188(.a(new_n282), .b(new_n283), .c(new_n278), .o(new_n284));
  aobi12aa1n03x7               g189(.a(new_n281), .b(new_n284), .c(new_n275), .out0(\s[25] ));
  tech160nm_fixorc02aa1n03p5x5 g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  nor042aa1n06x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n279), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n287), .o1(new_n290));
  aoai13aa1n02x7               g195(.a(new_n290), .b(new_n283), .c(new_n275), .d(new_n289), .o1(new_n291));
  aoi022aa1n02x7               g196(.a(new_n291), .b(new_n286), .c(new_n281), .d(new_n288), .o1(\s[26] ));
  and002aa1n12x5               g197(.a(new_n286), .b(new_n280), .o(new_n293));
  nano32aa1n03x7               g198(.a(new_n254), .b(new_n293), .c(new_n257), .d(new_n273), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n195), .c(new_n160), .d(new_n186), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n293), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[26] ), .b(\b[25] ), .c(new_n290), .carry(new_n297));
  aoai13aa1n04x5               g202(.a(new_n297), .b(new_n296), .c(new_n282), .d(new_n278), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n298), .c(new_n196), .d(new_n294), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n297), .o1(new_n301));
  aoi112aa1n02x5               g206(.a(new_n299), .b(new_n301), .c(new_n279), .d(new_n293), .o1(new_n302));
  aobi12aa1n03x7               g207(.a(new_n300), .b(new_n302), .c(new_n295), .out0(\s[27] ));
  tech160nm_fixorc02aa1n02p5x5 g208(.a(\a[28] ), .b(\b[27] ), .out0(new_n304));
  norp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n304), .b(new_n305), .o1(new_n306));
  tech160nm_fiaoi012aa1n05x5   g211(.a(new_n301), .b(new_n279), .c(new_n293), .o1(new_n307));
  inv000aa1n03x5               g212(.a(new_n305), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n299), .o1(new_n309));
  aoai13aa1n02x7               g214(.a(new_n308), .b(new_n309), .c(new_n307), .d(new_n295), .o1(new_n310));
  aoi022aa1n02x7               g215(.a(new_n310), .b(new_n304), .c(new_n300), .d(new_n306), .o1(\s[28] ));
  and002aa1n02x5               g216(.a(new_n304), .b(new_n299), .o(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n298), .c(new_n196), .d(new_n294), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n312), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n308), .carry(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n314), .c(new_n307), .d(new_n295), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .out0(new_n317));
  norb02aa1n02x5               g222(.a(new_n315), .b(new_n317), .out0(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n313), .d(new_n318), .o1(\s[29] ));
  xnrb03aa1n02x5               g224(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g225(.a(new_n309), .b(new_n304), .c(new_n317), .out0(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n298), .c(new_n196), .d(new_n294), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n321), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .carry(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n323), .c(new_n307), .d(new_n295), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .out0(new_n326));
  norb02aa1n02x5               g231(.a(new_n324), .b(new_n326), .out0(new_n327));
  aoi022aa1n03x5               g232(.a(new_n325), .b(new_n326), .c(new_n322), .d(new_n327), .o1(\s[30] ));
  nano32aa1n02x5               g233(.a(new_n309), .b(new_n326), .c(new_n304), .d(new_n317), .out0(new_n329));
  aoai13aa1n02x5               g234(.a(new_n329), .b(new_n298), .c(new_n196), .d(new_n294), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[31] ), .b(\b[30] ), .out0(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n324), .carry(new_n332));
  norb02aa1n02x5               g237(.a(new_n332), .b(new_n331), .out0(new_n333));
  inv000aa1n02x5               g238(.a(new_n329), .o1(new_n334));
  aoai13aa1n03x5               g239(.a(new_n332), .b(new_n334), .c(new_n307), .d(new_n295), .o1(new_n335));
  aoi022aa1n03x5               g240(.a(new_n335), .b(new_n331), .c(new_n330), .d(new_n333), .o1(\s[31] ));
  orn002aa1n02x5               g241(.a(\a[2] ), .b(\b[1] ), .o(new_n337));
  nanp02aa1n02x5               g242(.a(\b[1] ), .b(\a[2] ), .o1(new_n338));
  nanb03aa1n02x5               g243(.a(new_n99), .b(new_n337), .c(new_n338), .out0(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n106), .b(new_n339), .c(new_n337), .out0(\s[3] ));
  aoai13aa1n02x5               g245(.a(new_n103), .b(new_n104), .c(new_n100), .d(new_n105), .o1(new_n341));
  aoi112aa1n02x5               g246(.a(new_n104), .b(new_n103), .c(new_n100), .d(new_n105), .o1(new_n342));
  norb02aa1n02x5               g247(.a(new_n341), .b(new_n342), .out0(\s[4] ));
  xorc02aa1n02x5               g248(.a(\a[5] ), .b(\b[4] ), .out0(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n344), .b(new_n107), .c(new_n108), .out0(\s[5] ));
  nanp02aa1n03x5               g250(.a(new_n109), .b(new_n344), .o1(new_n346));
  xorc02aa1n02x5               g251(.a(\a[6] ), .b(\b[5] ), .out0(new_n347));
  oaoi13aa1n02x5               g252(.a(new_n347), .b(new_n346), .c(\a[5] ), .d(\b[4] ), .o1(new_n348));
  nona22aa1n03x5               g253(.a(new_n346), .b(new_n156), .c(new_n115), .out0(new_n349));
  nanb02aa1n02x5               g254(.a(new_n348), .b(new_n349), .out0(\s[6] ));
  nona23aa1n03x5               g255(.a(new_n349), .b(new_n113), .c(new_n111), .d(new_n156), .out0(new_n351));
  aboi22aa1n03x5               g256(.a(new_n111), .b(new_n113), .c(new_n349), .d(new_n116), .out0(new_n352));
  norb02aa1n02x5               g257(.a(new_n351), .b(new_n352), .out0(\s[7] ));
  orn002aa1n02x5               g258(.a(\a[7] ), .b(\b[6] ), .o(new_n354));
  norb02aa1n02x5               g259(.a(new_n110), .b(new_n112), .out0(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n355), .b(new_n351), .c(new_n354), .out0(\s[8] ));
  nanp02aa1n02x5               g261(.a(new_n109), .b(new_n119), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n358));
  aoi113aa1n02x5               g263(.a(new_n124), .b(new_n158), .c(new_n120), .d(new_n358), .e(new_n115), .o1(new_n359));
  aoi022aa1n02x5               g264(.a(new_n160), .b(new_n124), .c(new_n357), .d(new_n359), .o1(\s[9] ));
endmodule


