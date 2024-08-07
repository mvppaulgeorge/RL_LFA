// Benchmark "adder" written by ABC on Thu Jul 18 08:33:01 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n340, new_n342, new_n343, new_n346, new_n348,
    new_n349, new_n350, new_n352, new_n353, new_n355, new_n356;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  inv000aa1n02x5               g004(.a(new_n99), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand42aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aob012aa1n06x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  nor002aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n02x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor002aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nand42aa1n03x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n06x4               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nand23aa1n04x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\a[3] ), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\b[2] ), .o1(new_n112));
  nand02aa1d08x5               g017(.a(new_n112), .b(new_n111), .o1(new_n113));
  oaoi03aa1n12x5               g018(.a(\a[4] ), .b(\b[3] ), .c(new_n113), .o1(new_n114));
  inv000aa1d42x5               g019(.a(new_n114), .o1(new_n115));
  nor022aa1n04x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand42aa1n08x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor022aa1n06x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nand22aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1n06x5               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  norp02aa1n02x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  nand22aa1n03x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nanb02aa1n03x5               g027(.a(new_n121), .b(new_n122), .out0(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[4] ), .b(\a[5] ), .out0(new_n124));
  nona22aa1n02x4               g029(.a(new_n120), .b(new_n124), .c(new_n123), .out0(new_n125));
  nona22aa1n02x4               g030(.a(new_n122), .b(\b[4] ), .c(\a[5] ), .out0(new_n126));
  oai012aa1n02x5               g031(.a(new_n126), .b(\b[5] ), .c(\a[6] ), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n128));
  aobi12aa1n09x5               g033(.a(new_n128), .b(new_n120), .c(new_n127), .out0(new_n129));
  aoai13aa1n12x5               g034(.a(new_n129), .b(new_n125), .c(new_n110), .d(new_n115), .o1(new_n130));
  nand42aa1n03x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  norb02aa1n03x4               g036(.a(new_n131), .b(new_n97), .out0(new_n132));
  nanp02aa1n02x5               g037(.a(new_n130), .b(new_n132), .o1(new_n133));
  nor002aa1n06x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nand22aa1n09x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  norb02aa1n09x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n133), .c(new_n98), .out0(\s[10] ));
  nano23aa1n02x5               g042(.a(new_n97), .b(new_n134), .c(new_n135), .d(new_n131), .out0(new_n138));
  aoi012aa1d24x5               g043(.a(new_n134), .b(new_n97), .c(new_n135), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  nor022aa1n08x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand02aa1n03x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n03x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoai13aa1n06x5               g048(.a(new_n143), .b(new_n140), .c(new_n130), .d(new_n138), .o1(new_n144));
  aoi112aa1n02x5               g049(.a(new_n143), .b(new_n140), .c(new_n130), .d(new_n138), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(\s[11] ));
  norp02aa1n09x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand02aa1n04x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  aoib12aa1n02x5               g054(.a(new_n141), .b(new_n148), .c(new_n147), .out0(new_n150));
  oai012aa1n02x5               g055(.a(new_n144), .b(\b[10] ), .c(\a[11] ), .o1(new_n151));
  aoi022aa1n02x5               g056(.a(new_n151), .b(new_n149), .c(new_n144), .d(new_n150), .o1(\s[12] ));
  nona23aa1n09x5               g057(.a(new_n148), .b(new_n142), .c(new_n141), .d(new_n147), .out0(new_n153));
  nano22aa1n03x7               g058(.a(new_n153), .b(new_n132), .c(new_n136), .out0(new_n154));
  tech160nm_fiaoi012aa1n03p5x5 g059(.a(new_n147), .b(new_n141), .c(new_n148), .o1(new_n155));
  oai012aa1n12x5               g060(.a(new_n155), .b(new_n153), .c(new_n139), .o1(new_n156));
  xnrc02aa1n12x5               g061(.a(\b[12] ), .b(\a[13] ), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n156), .c(new_n130), .d(new_n154), .o1(new_n159));
  oai112aa1n02x5               g064(.a(new_n155), .b(new_n157), .c(new_n153), .d(new_n139), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n130), .c(new_n154), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n159), .b(new_n161), .out0(\s[13] ));
  orn002aa1n02x5               g067(.a(\a[13] ), .b(\b[12] ), .o(new_n163));
  xnrc02aa1n12x5               g068(.a(\b[13] ), .b(\a[14] ), .out0(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n159), .c(new_n163), .out0(\s[14] ));
  norp02aa1n02x5               g071(.a(new_n164), .b(new_n157), .o1(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n156), .c(new_n130), .d(new_n154), .o1(new_n168));
  nor002aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  aoi112aa1n09x5               g074(.a(\b[12] ), .b(\a[13] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n170));
  nor042aa1n06x5               g075(.a(new_n170), .b(new_n169), .o1(new_n171));
  nor002aa1d32x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanp02aa1n04x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nanb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n168), .c(new_n171), .out0(\s[15] ));
  aob012aa1n02x5               g081(.a(new_n175), .b(new_n168), .c(new_n171), .out0(new_n177));
  nor042aa1n04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanp02aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aoib12aa1n02x5               g085(.a(new_n172), .b(new_n179), .c(new_n178), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n172), .o1(new_n182));
  aoai13aa1n03x5               g087(.a(new_n182), .b(new_n174), .c(new_n168), .d(new_n171), .o1(new_n183));
  aoi022aa1n03x5               g088(.a(new_n183), .b(new_n180), .c(new_n177), .d(new_n181), .o1(\s[16] ));
  nano23aa1n03x7               g089(.a(new_n172), .b(new_n178), .c(new_n179), .d(new_n173), .out0(new_n185));
  nona22aa1n02x4               g090(.a(new_n185), .b(new_n164), .c(new_n157), .out0(new_n186));
  nano32aa1n03x7               g091(.a(new_n186), .b(new_n149), .c(new_n143), .d(new_n138), .out0(new_n187));
  nanp02aa1n02x5               g092(.a(new_n130), .b(new_n187), .o1(new_n188));
  tech160nm_fiaoi012aa1n05x5   g093(.a(new_n99), .b(new_n101), .c(new_n102), .o1(new_n189));
  nanb02aa1n02x5               g094(.a(new_n104), .b(new_n105), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(new_n113), .b(new_n108), .o1(new_n191));
  norp03aa1n02x5               g096(.a(new_n189), .b(new_n190), .c(new_n191), .o1(new_n192));
  nona23aa1n03x5               g097(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n193));
  nor003aa1n02x5               g098(.a(new_n193), .b(new_n123), .c(new_n124), .o1(new_n194));
  oai012aa1n06x5               g099(.a(new_n194), .b(new_n192), .c(new_n114), .o1(new_n195));
  nona23aa1n09x5               g100(.a(new_n179), .b(new_n173), .c(new_n172), .d(new_n178), .out0(new_n196));
  nona32aa1n03x5               g101(.a(new_n154), .b(new_n196), .c(new_n164), .d(new_n157), .out0(new_n197));
  norp03aa1n06x5               g102(.a(new_n196), .b(new_n164), .c(new_n157), .o1(new_n198));
  aoi012aa1n06x5               g103(.a(new_n178), .b(new_n172), .c(new_n179), .o1(new_n199));
  tech160nm_fioai012aa1n04x5   g104(.a(new_n199), .b(new_n196), .c(new_n171), .o1(new_n200));
  aoi012aa1n12x5               g105(.a(new_n200), .b(new_n156), .c(new_n198), .o1(new_n201));
  aoai13aa1n12x5               g106(.a(new_n201), .b(new_n197), .c(new_n195), .d(new_n129), .o1(new_n202));
  xorc02aa1n12x5               g107(.a(\a[17] ), .b(\b[16] ), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n171), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n199), .out0(new_n205));
  aoi122aa1n02x5               g110(.a(new_n205), .b(new_n204), .c(new_n185), .d(new_n156), .e(new_n198), .o1(new_n206));
  aoi022aa1n02x5               g111(.a(new_n202), .b(new_n203), .c(new_n188), .d(new_n206), .o1(\s[17] ));
  nor042aa1n04x5               g112(.a(\b[16] ), .b(\a[17] ), .o1(new_n208));
  nor042aa1n04x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nanp02aa1n12x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  norb02aa1n06x4               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aoi112aa1n02x5               g116(.a(new_n208), .b(new_n211), .c(new_n202), .d(new_n203), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n211), .b(new_n208), .c(new_n202), .d(new_n203), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(\s[18] ));
  and002aa1n06x5               g119(.a(new_n203), .b(new_n211), .o(new_n215));
  aoi012aa1d24x5               g120(.a(new_n209), .b(new_n208), .c(new_n210), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[19] ), .b(\b[18] ), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n217), .c(new_n202), .d(new_n215), .o1(new_n219));
  aoi112aa1n02x5               g124(.a(new_n218), .b(new_n217), .c(new_n202), .d(new_n215), .o1(new_n220));
  norb02aa1n03x4               g125(.a(new_n219), .b(new_n220), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n02x5               g127(.a(\a[20] ), .b(\b[19] ), .out0(new_n223));
  norp02aa1n02x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  norp02aa1n02x5               g129(.a(new_n223), .b(new_n224), .o1(new_n225));
  tech160nm_fioai012aa1n03p5x5 g130(.a(new_n219), .b(\b[18] ), .c(\a[19] ), .o1(new_n226));
  aoi022aa1n02x7               g131(.a(new_n226), .b(new_n223), .c(new_n219), .d(new_n225), .o1(\s[20] ));
  xnrc02aa1n02x5               g132(.a(\b[19] ), .b(\a[20] ), .out0(new_n228));
  nano32aa1n02x4               g133(.a(new_n228), .b(new_n218), .c(new_n203), .d(new_n211), .out0(new_n229));
  xnrc02aa1n02x5               g134(.a(\b[18] ), .b(\a[19] ), .out0(new_n230));
  norp02aa1n02x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  aoi012aa1n02x7               g137(.a(new_n231), .b(new_n224), .c(new_n232), .o1(new_n233));
  oai013aa1n03x5               g138(.a(new_n233), .b(new_n230), .c(new_n228), .d(new_n216), .o1(new_n234));
  nor042aa1n04x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  nand42aa1n04x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nanb02aa1n09x5               g141(.a(new_n235), .b(new_n236), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n234), .c(new_n202), .d(new_n229), .o1(new_n239));
  nanb03aa1n02x5               g144(.a(new_n216), .b(new_n223), .c(new_n218), .out0(new_n240));
  nanp03aa1n02x5               g145(.a(new_n240), .b(new_n233), .c(new_n237), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n202), .c(new_n229), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n239), .b(new_n242), .out0(\s[21] ));
  nor042aa1n04x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nand22aa1n09x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nanb02aa1n06x5               g150(.a(new_n244), .b(new_n245), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  aoib12aa1n02x5               g152(.a(new_n235), .b(new_n245), .c(new_n244), .out0(new_n248));
  tech160nm_fioai012aa1n03p5x5 g153(.a(new_n239), .b(\b[20] ), .c(\a[21] ), .o1(new_n249));
  aoi022aa1n02x7               g154(.a(new_n249), .b(new_n247), .c(new_n239), .d(new_n248), .o1(\s[22] ));
  nanb03aa1n02x5               g155(.a(new_n139), .b(new_n149), .c(new_n143), .out0(new_n251));
  aobi12aa1n02x5               g156(.a(new_n199), .b(new_n204), .c(new_n185), .out0(new_n252));
  aoai13aa1n02x7               g157(.a(new_n252), .b(new_n186), .c(new_n251), .d(new_n155), .o1(new_n253));
  norp02aa1n02x5               g158(.a(new_n228), .b(new_n230), .o1(new_n254));
  nona23aa1d18x5               g159(.a(new_n245), .b(new_n236), .c(new_n235), .d(new_n244), .out0(new_n255));
  nano32aa1n02x4               g160(.a(new_n255), .b(new_n254), .c(new_n211), .d(new_n203), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n253), .c(new_n130), .d(new_n187), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n255), .o1(new_n258));
  ao0012aa1n03x7               g163(.a(new_n244), .b(new_n235), .c(new_n245), .o(new_n259));
  aoi012aa1n02x5               g164(.a(new_n259), .b(new_n234), .c(new_n258), .o1(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[22] ), .b(\a[23] ), .out0(new_n261));
  inv000aa1n04x5               g166(.a(new_n261), .o1(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n257), .c(new_n260), .out0(\s[23] ));
  aob012aa1n02x5               g168(.a(new_n262), .b(new_n257), .c(new_n260), .out0(new_n264));
  xorc02aa1n12x5               g169(.a(\a[24] ), .b(\b[23] ), .out0(new_n265));
  nor042aa1n06x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n265), .b(new_n266), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n266), .o1(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n261), .c(new_n257), .d(new_n260), .o1(new_n269));
  aoi022aa1n03x5               g174(.a(new_n269), .b(new_n265), .c(new_n264), .d(new_n267), .o1(\s[24] ));
  nona23aa1n09x5               g175(.a(new_n262), .b(new_n265), .c(new_n246), .d(new_n237), .out0(new_n271));
  nano32aa1n03x7               g176(.a(new_n271), .b(new_n254), .c(new_n211), .d(new_n203), .out0(new_n272));
  oaoi03aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n268), .o1(new_n273));
  aoi013aa1n02x5               g178(.a(new_n273), .b(new_n262), .c(new_n259), .d(new_n265), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n271), .c(new_n240), .d(new_n233), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[25] ), .b(\b[24] ), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n275), .c(new_n202), .d(new_n272), .o1(new_n277));
  aoai13aa1n02x5               g182(.a(new_n272), .b(new_n253), .c(new_n130), .d(new_n187), .o1(new_n278));
  norb03aa1n06x5               g183(.a(new_n265), .b(new_n255), .c(new_n261), .out0(new_n279));
  nanp02aa1n03x5               g184(.a(new_n234), .b(new_n279), .o1(new_n280));
  aoi113aa1n02x5               g185(.a(new_n276), .b(new_n273), .c(new_n262), .d(new_n259), .e(new_n265), .o1(new_n281));
  and003aa1n03x7               g186(.a(new_n278), .b(new_n281), .c(new_n280), .o(new_n282));
  norb02aa1n03x4               g187(.a(new_n277), .b(new_n282), .out0(\s[25] ));
  xorc02aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .out0(new_n284));
  nor042aa1n02x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n284), .b(new_n285), .o1(new_n286));
  inv000aa1d42x5               g191(.a(\a[25] ), .o1(new_n287));
  oaib12aa1n06x5               g192(.a(new_n277), .b(\b[24] ), .c(new_n287), .out0(new_n288));
  aoi022aa1n02x7               g193(.a(new_n288), .b(new_n284), .c(new_n277), .d(new_n286), .o1(\s[26] ));
  inv040aa1d32x5               g194(.a(\a[26] ), .o1(new_n290));
  xroi22aa1d06x4               g195(.a(new_n287), .b(\b[24] ), .c(new_n290), .d(\b[25] ), .out0(new_n291));
  nano32aa1n03x7               g196(.a(new_n271), .b(new_n215), .c(new_n291), .d(new_n254), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n253), .c(new_n130), .d(new_n187), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n291), .o1(new_n294));
  inv000aa1d42x5               g199(.a(\b[25] ), .o1(new_n295));
  tech160nm_fioaoi03aa1n02p5x5 g200(.a(new_n290), .b(new_n295), .c(new_n285), .o1(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n294), .c(new_n280), .d(new_n274), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n297), .c(new_n202), .d(new_n292), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n296), .o1(new_n300));
  aoi112aa1n02x5               g205(.a(new_n298), .b(new_n300), .c(new_n275), .d(new_n291), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n299), .b(new_n301), .c(new_n293), .out0(\s[27] ));
  xorc02aa1n02x5               g207(.a(\a[28] ), .b(\b[27] ), .out0(new_n303));
  norp02aa1n02x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  norp02aa1n02x5               g209(.a(new_n303), .b(new_n304), .o1(new_n305));
  inv000aa1d42x5               g210(.a(\a[27] ), .o1(new_n306));
  oaib12aa1n06x5               g211(.a(new_n299), .b(\b[26] ), .c(new_n306), .out0(new_n307));
  aoi022aa1n02x7               g212(.a(new_n307), .b(new_n303), .c(new_n299), .d(new_n305), .o1(\s[28] ));
  inv000aa1d42x5               g213(.a(\a[28] ), .o1(new_n309));
  xroi22aa1d04x5               g214(.a(new_n306), .b(\b[26] ), .c(new_n309), .d(\b[27] ), .out0(new_n310));
  aoai13aa1n06x5               g215(.a(new_n310), .b(new_n297), .c(new_n202), .d(new_n292), .o1(new_n311));
  inv000aa1d42x5               g216(.a(\b[27] ), .o1(new_n312));
  oao003aa1n09x5               g217(.a(new_n309), .b(new_n312), .c(new_n304), .carry(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  nanp02aa1n02x5               g219(.a(new_n311), .b(new_n314), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  norp02aa1n02x5               g221(.a(new_n313), .b(new_n316), .o1(new_n317));
  aoi022aa1n02x5               g222(.a(new_n315), .b(new_n316), .c(new_n311), .d(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g224(.a(new_n298), .b(new_n316), .c(new_n303), .o(new_n320));
  aoai13aa1n06x5               g225(.a(new_n320), .b(new_n297), .c(new_n202), .d(new_n292), .o1(new_n321));
  inv000aa1d42x5               g226(.a(\a[29] ), .o1(new_n322));
  inv000aa1d42x5               g227(.a(\b[28] ), .o1(new_n323));
  oaoi03aa1n02x5               g228(.a(new_n322), .b(new_n323), .c(new_n313), .o1(new_n324));
  nanp02aa1n03x5               g229(.a(new_n321), .b(new_n324), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .out0(new_n326));
  oabi12aa1n02x5               g231(.a(new_n326), .b(\a[29] ), .c(\b[28] ), .out0(new_n327));
  oaoi13aa1n02x5               g232(.a(new_n327), .b(new_n313), .c(new_n322), .d(new_n323), .o1(new_n328));
  aoi022aa1n02x7               g233(.a(new_n325), .b(new_n326), .c(new_n321), .d(new_n328), .o1(\s[30] ));
  nanb02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  nanb02aa1n02x5               g235(.a(\a[31] ), .b(\b[30] ), .out0(new_n331));
  and003aa1n02x5               g236(.a(new_n310), .b(new_n326), .c(new_n316), .o(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n297), .c(new_n202), .d(new_n292), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n324), .carry(new_n334));
  aoi022aa1n02x7               g239(.a(new_n333), .b(new_n334), .c(new_n331), .d(new_n330), .o1(new_n335));
  aoi012aa1n02x5               g240(.a(new_n300), .b(new_n275), .c(new_n291), .o1(new_n336));
  aobi12aa1n06x5               g241(.a(new_n332), .b(new_n293), .c(new_n336), .out0(new_n337));
  nano32aa1n03x5               g242(.a(new_n337), .b(new_n334), .c(new_n330), .d(new_n331), .out0(new_n338));
  norp02aa1n03x5               g243(.a(new_n335), .b(new_n338), .o1(\s[31] ));
  aoi112aa1n02x5               g244(.a(new_n109), .b(new_n99), .c(new_n101), .d(new_n102), .o1(new_n340));
  aoi012aa1n02x5               g245(.a(new_n340), .b(new_n103), .c(new_n109), .o1(\s[3] ));
  aoi013aa1n02x4               g246(.a(new_n114), .b(new_n103), .c(new_n106), .d(new_n109), .o1(new_n342));
  aoi112aa1n02x5               g247(.a(new_n107), .b(new_n106), .c(new_n103), .d(new_n109), .o1(new_n343));
  oab012aa1n02x4               g248(.a(new_n343), .b(new_n342), .c(new_n104), .out0(\s[4] ));
  xobna2aa1n03x5               g249(.a(new_n124), .b(new_n110), .c(new_n115), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g250(.a(\a[5] ), .b(\b[4] ), .c(new_n342), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g252(.a(new_n119), .b(new_n118), .out0(new_n348));
  aoai13aa1n03x5               g253(.a(new_n348), .b(new_n121), .c(new_n346), .d(new_n122), .o1(new_n349));
  aoi112aa1n02x5               g254(.a(new_n348), .b(new_n121), .c(new_n346), .d(new_n122), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n349), .b(new_n350), .out0(\s[7] ));
  norb02aa1n02x5               g256(.a(new_n117), .b(new_n116), .out0(new_n352));
  orn002aa1n02x5               g257(.a(\a[7] ), .b(\b[6] ), .o(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n352), .b(new_n349), .c(new_n353), .out0(\s[8] ));
  oaib12aa1n02x5               g259(.a(new_n128), .b(new_n97), .c(new_n131), .out0(new_n355));
  aoi012aa1n02x5               g260(.a(new_n355), .b(new_n120), .c(new_n127), .o1(new_n356));
  aoi022aa1n02x5               g261(.a(new_n130), .b(new_n132), .c(new_n195), .d(new_n356), .o1(\s[9] ));
endmodule


