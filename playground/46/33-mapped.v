// Benchmark "adder" written by ABC on Thu Jul 18 11:51:54 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n339, new_n341, new_n342, new_n343, new_n345, new_n346,
    new_n347, new_n348, new_n349, new_n351, new_n353, new_n354, new_n355,
    new_n356, new_n358;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  oa0022aa1n02x5               g002(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n98));
  tech160nm_fixnrc02aa1n04x5   g003(.a(\b[2] ), .b(\a[3] ), .out0(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  tech160nm_fioai012aa1n05x5   g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  oai012aa1n12x5               g008(.a(new_n98), .b(new_n99), .c(new_n103), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  norp02aa1n04x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  aoi012aa1n02x5               g011(.a(new_n106), .b(\a[5] ), .c(\b[4] ), .o1(new_n107));
  nor022aa1n12x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n108), .b(\a[4] ), .c(\b[3] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nor002aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor022aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n112), .b(new_n110), .c(new_n113), .d(new_n111), .out0(new_n114));
  nano32aa1n03x7               g019(.a(new_n114), .b(new_n109), .c(new_n107), .d(new_n105), .out0(new_n115));
  nanp02aa1n06x5               g020(.a(new_n115), .b(new_n104), .o1(new_n116));
  inv040aa1d32x5               g021(.a(\a[5] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\b[4] ), .o1(new_n118));
  nanp02aa1n12x5               g023(.a(new_n118), .b(new_n117), .o1(new_n119));
  oaoi03aa1n06x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  aoi022aa1n02x5               g025(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n121));
  oaoi13aa1n09x5               g026(.a(new_n106), .b(new_n121), .c(new_n120), .d(new_n108), .o1(new_n122));
  nand02aa1d06x5               g027(.a(new_n116), .b(new_n122), .o1(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  nor042aa1n04x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1n20x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n03x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n97), .c(new_n123), .d(new_n125), .o1(new_n129));
  aoi112aa1n02x5               g034(.a(new_n128), .b(new_n97), .c(new_n123), .d(new_n125), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n129), .b(new_n130), .out0(\s[10] ));
  oai012aa1n02x5               g036(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1d27x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n129), .c(new_n132), .out0(\s[11] ));
  aob012aa1n03x5               g041(.a(new_n135), .b(new_n129), .c(new_n132), .out0(new_n137));
  nor002aa1d24x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1d08x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n138), .o1(new_n141));
  aoi012aa1n02x5               g046(.a(new_n133), .b(new_n141), .c(new_n139), .o1(new_n142));
  inv030aa1n03x5               g047(.a(new_n133), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n135), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n143), .b(new_n144), .c(new_n129), .d(new_n132), .o1(new_n145));
  aboi22aa1n03x5               g050(.a(new_n140), .b(new_n145), .c(new_n137), .d(new_n142), .out0(\s[12] ));
  inv040aa1n03x5               g051(.a(new_n128), .o1(new_n147));
  nona23aa1n12x5               g052(.a(new_n139), .b(new_n134), .c(new_n133), .d(new_n138), .out0(new_n148));
  nor043aa1d12x5               g053(.a(new_n148), .b(new_n147), .c(new_n124), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  nanb03aa1n09x5               g055(.a(new_n138), .b(new_n139), .c(new_n134), .out0(new_n151));
  oai112aa1n06x5               g056(.a(new_n143), .b(new_n127), .c(new_n126), .d(new_n97), .o1(new_n152));
  aoi012aa1n12x5               g057(.a(new_n138), .b(new_n133), .c(new_n139), .o1(new_n153));
  oai012aa1d24x5               g058(.a(new_n153), .b(new_n152), .c(new_n151), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n150), .c(new_n116), .d(new_n122), .o1(new_n156));
  nor042aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  aoi112aa1n02x5               g064(.a(new_n159), .b(new_n154), .c(new_n123), .d(new_n149), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n156), .c(new_n159), .o1(\s[13] ));
  nor042aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  aoi112aa1n02x5               g069(.a(new_n157), .b(new_n164), .c(new_n156), .d(new_n159), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n164), .b(new_n157), .c(new_n156), .d(new_n158), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(\s[14] ));
  nano23aa1n06x5               g072(.a(new_n157), .b(new_n162), .c(new_n163), .d(new_n158), .out0(new_n168));
  tech160nm_fioai012aa1n03p5x5 g073(.a(new_n163), .b(new_n162), .c(new_n157), .o1(new_n169));
  inv000aa1n02x5               g074(.a(new_n169), .o1(new_n170));
  nor002aa1n03x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanp02aa1n03x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n170), .c(new_n156), .d(new_n168), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(new_n174), .b(new_n170), .c(new_n156), .d(new_n168), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(\s[15] ));
  norp02aa1n04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  tech160nm_finand02aa1n03p5x5 g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(new_n180));
  aoib12aa1n02x5               g085(.a(new_n171), .b(new_n179), .c(new_n178), .out0(new_n181));
  tech160nm_fioai012aa1n03p5x5 g086(.a(new_n175), .b(\b[14] ), .c(\a[15] ), .o1(new_n182));
  aboi22aa1n03x5               g087(.a(new_n180), .b(new_n182), .c(new_n175), .d(new_n181), .out0(\s[16] ));
  oaoi13aa1n03x5               g088(.a(new_n108), .b(new_n112), .c(new_n111), .d(new_n113), .o1(new_n184));
  obai22aa1n02x7               g089(.a(new_n121), .b(new_n184), .c(\a[8] ), .d(\b[7] ), .out0(new_n185));
  tech160nm_fiaoi012aa1n04x5   g090(.a(new_n185), .b(new_n115), .c(new_n104), .o1(new_n186));
  nona23aa1n02x4               g091(.a(new_n163), .b(new_n158), .c(new_n157), .d(new_n162), .out0(new_n187));
  nona23aa1d18x5               g092(.a(new_n179), .b(new_n172), .c(new_n171), .d(new_n178), .out0(new_n188));
  nona22aa1n09x5               g093(.a(new_n149), .b(new_n187), .c(new_n188), .out0(new_n189));
  inv000aa1d42x5               g094(.a(new_n188), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n170), .c(new_n154), .d(new_n168), .o1(new_n191));
  tech160nm_fiaoi012aa1n03p5x5 g096(.a(new_n178), .b(new_n171), .c(new_n179), .o1(new_n192));
  oai112aa1n06x5               g097(.a(new_n191), .b(new_n192), .c(new_n186), .d(new_n189), .o1(new_n193));
  xorc02aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  aoi012aa1n12x5               g099(.a(new_n189), .b(new_n116), .c(new_n122), .o1(new_n195));
  nano23aa1n02x4               g100(.a(new_n194), .b(new_n195), .c(new_n191), .d(new_n192), .out0(new_n196));
  aoi012aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n194), .o1(\s[17] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(\b[16] ), .b(new_n198), .out0(new_n199));
  nano22aa1n03x5               g104(.a(new_n138), .b(new_n134), .c(new_n139), .out0(new_n200));
  oai012aa1n02x5               g105(.a(new_n127), .b(\b[10] ), .c(\a[11] ), .o1(new_n201));
  oab012aa1n02x4               g106(.a(new_n201), .b(new_n97), .c(new_n126), .out0(new_n202));
  inv020aa1n02x5               g107(.a(new_n153), .o1(new_n203));
  aoai13aa1n06x5               g108(.a(new_n168), .b(new_n203), .c(new_n202), .d(new_n200), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n192), .b(new_n188), .c(new_n204), .d(new_n169), .o1(new_n205));
  oaih12aa1n02x5               g110(.a(new_n194), .b(new_n205), .c(new_n195), .o1(new_n206));
  nor022aa1n12x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nand22aa1n06x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  norb02aa1n06x4               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n206), .c(new_n199), .out0(\s[18] ));
  and002aa1n02x5               g115(.a(new_n194), .b(new_n209), .o(new_n211));
  tech160nm_fioai012aa1n05x5   g116(.a(new_n211), .b(new_n205), .c(new_n195), .o1(new_n212));
  oaoi03aa1n02x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  nor002aa1d32x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nanp02aa1n04x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1n06x4               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n212), .c(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g124(.a(new_n217), .b(new_n213), .c(new_n193), .d(new_n211), .o1(new_n220));
  nor042aa1n06x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand22aa1n09x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[19] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[18] ), .o1(new_n225));
  aboi22aa1n03x5               g130(.a(new_n221), .b(new_n222), .c(new_n224), .d(new_n225), .out0(new_n226));
  inv040aa1n02x5               g131(.a(new_n215), .o1(new_n227));
  inv000aa1n03x5               g132(.a(new_n217), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n227), .b(new_n228), .c(new_n212), .d(new_n214), .o1(new_n229));
  aoi022aa1n03x5               g134(.a(new_n229), .b(new_n223), .c(new_n220), .d(new_n226), .o1(\s[20] ));
  nano32aa1n03x7               g135(.a(new_n228), .b(new_n194), .c(new_n223), .d(new_n209), .out0(new_n231));
  tech160nm_fioai012aa1n05x5   g136(.a(new_n231), .b(new_n205), .c(new_n195), .o1(new_n232));
  nanb03aa1n09x5               g137(.a(new_n221), .b(new_n222), .c(new_n216), .out0(new_n233));
  nor042aa1n04x5               g138(.a(\b[16] ), .b(\a[17] ), .o1(new_n234));
  oai112aa1n06x5               g139(.a(new_n227), .b(new_n208), .c(new_n207), .d(new_n234), .o1(new_n235));
  aoi012aa1n09x5               g140(.a(new_n221), .b(new_n215), .c(new_n222), .o1(new_n236));
  oai012aa1n18x5               g141(.a(new_n236), .b(new_n235), .c(new_n233), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[20] ), .b(\a[21] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  xnbna2aa1n03x5               g145(.a(new_n240), .b(new_n232), .c(new_n238), .out0(\s[21] ));
  aoai13aa1n03x5               g146(.a(new_n240), .b(new_n237), .c(new_n193), .d(new_n231), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[21] ), .b(\a[22] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  nor042aa1n09x5               g149(.a(\b[20] ), .b(\a[21] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n243), .b(new_n245), .out0(new_n246));
  inv000aa1n06x5               g151(.a(new_n245), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n239), .c(new_n232), .d(new_n238), .o1(new_n248));
  aoi022aa1n03x5               g153(.a(new_n248), .b(new_n244), .c(new_n242), .d(new_n246), .o1(\s[22] ));
  nor042aa1n06x5               g154(.a(new_n243), .b(new_n239), .o1(new_n250));
  and002aa1n02x5               g155(.a(new_n231), .b(new_n250), .o(new_n251));
  oaih12aa1n02x5               g156(.a(new_n251), .b(new_n205), .c(new_n195), .o1(new_n252));
  oao003aa1n06x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .carry(new_n253));
  inv000aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  tech160nm_fiaoi012aa1n05x5   g159(.a(new_n254), .b(new_n237), .c(new_n250), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  xorc02aa1n12x5               g161(.a(\a[23] ), .b(\b[22] ), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n256), .c(new_n193), .d(new_n251), .o1(new_n258));
  aoi112aa1n02x5               g163(.a(new_n257), .b(new_n254), .c(new_n237), .d(new_n250), .o1(new_n259));
  aobi12aa1n03x7               g164(.a(new_n258), .b(new_n259), .c(new_n252), .out0(\s[23] ));
  tech160nm_fixorc02aa1n02p5x5 g165(.a(\a[24] ), .b(\b[23] ), .out0(new_n261));
  nor042aa1n06x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  norp02aa1n02x5               g167(.a(new_n261), .b(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n262), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n257), .o1(new_n265));
  aoai13aa1n03x5               g170(.a(new_n264), .b(new_n265), .c(new_n252), .d(new_n255), .o1(new_n266));
  aoi022aa1n03x5               g171(.a(new_n266), .b(new_n261), .c(new_n258), .d(new_n263), .o1(\s[24] ));
  inv000aa1n02x5               g172(.a(new_n231), .o1(new_n268));
  and002aa1n06x5               g173(.a(new_n261), .b(new_n257), .o(new_n269));
  nano22aa1n02x5               g174(.a(new_n268), .b(new_n269), .c(new_n250), .out0(new_n270));
  oaih12aa1n02x5               g175(.a(new_n270), .b(new_n205), .c(new_n195), .o1(new_n271));
  nano22aa1n03x5               g176(.a(new_n221), .b(new_n216), .c(new_n222), .out0(new_n272));
  oai012aa1n02x5               g177(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .o1(new_n273));
  oab012aa1n02x5               g178(.a(new_n273), .b(new_n234), .c(new_n207), .out0(new_n274));
  inv000aa1n03x5               g179(.a(new_n236), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n250), .b(new_n275), .c(new_n274), .d(new_n272), .o1(new_n276));
  inv020aa1n03x5               g181(.a(new_n269), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[24] ), .b(\b[23] ), .c(new_n264), .carry(new_n278));
  aoai13aa1n12x5               g183(.a(new_n278), .b(new_n277), .c(new_n276), .d(new_n253), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n279), .c(new_n193), .d(new_n270), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n269), .b(new_n254), .c(new_n237), .d(new_n250), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n280), .o1(new_n283));
  and003aa1n02x5               g188(.a(new_n282), .b(new_n283), .c(new_n278), .o(new_n284));
  aobi12aa1n03x7               g189(.a(new_n281), .b(new_n284), .c(new_n271), .out0(\s[25] ));
  xorc02aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  nor042aa1n06x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n279), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n287), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n283), .c(new_n271), .d(new_n289), .o1(new_n291));
  aoi022aa1n03x5               g196(.a(new_n291), .b(new_n286), .c(new_n281), .d(new_n288), .o1(\s[26] ));
  and002aa1n12x5               g197(.a(new_n286), .b(new_n280), .o(new_n293));
  nano32aa1n03x7               g198(.a(new_n268), .b(new_n293), .c(new_n250), .d(new_n269), .out0(new_n294));
  oai012aa1n06x5               g199(.a(new_n294), .b(new_n205), .c(new_n195), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n293), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[26] ), .b(\b[25] ), .c(new_n290), .carry(new_n297));
  aoai13aa1n04x5               g202(.a(new_n297), .b(new_n296), .c(new_n282), .d(new_n278), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n298), .c(new_n193), .d(new_n294), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n297), .o1(new_n301));
  aoi112aa1n02x5               g206(.a(new_n299), .b(new_n301), .c(new_n279), .d(new_n293), .o1(new_n302));
  aobi12aa1n02x7               g207(.a(new_n300), .b(new_n302), .c(new_n295), .out0(\s[27] ));
  tech160nm_fixorc02aa1n02p5x5 g208(.a(\a[28] ), .b(\b[27] ), .out0(new_n304));
  norp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n304), .b(new_n305), .o1(new_n306));
  aoi012aa1n12x5               g211(.a(new_n301), .b(new_n279), .c(new_n293), .o1(new_n307));
  inv000aa1n03x5               g212(.a(new_n305), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n299), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n308), .b(new_n309), .c(new_n307), .d(new_n295), .o1(new_n310));
  aoi022aa1n02x7               g215(.a(new_n310), .b(new_n304), .c(new_n300), .d(new_n306), .o1(\s[28] ));
  and002aa1n02x5               g216(.a(new_n304), .b(new_n299), .o(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n298), .c(new_n193), .d(new_n294), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n312), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n308), .carry(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n314), .c(new_n307), .d(new_n295), .o1(new_n316));
  tech160nm_fixorc02aa1n02p5x5 g221(.a(\a[29] ), .b(\b[28] ), .out0(new_n317));
  norb02aa1n02x5               g222(.a(new_n315), .b(new_n317), .out0(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n313), .d(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g224(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g225(.a(new_n309), .b(new_n304), .c(new_n317), .out0(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n298), .c(new_n193), .d(new_n294), .o1(new_n322));
  inv000aa1n02x5               g227(.a(new_n321), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .carry(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n323), .c(new_n307), .d(new_n295), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .out0(new_n326));
  norb02aa1n02x5               g231(.a(new_n324), .b(new_n326), .out0(new_n327));
  aoi022aa1n03x5               g232(.a(new_n325), .b(new_n326), .c(new_n322), .d(new_n327), .o1(\s[30] ));
  xorc02aa1n02x5               g233(.a(\a[31] ), .b(\b[30] ), .out0(new_n329));
  nano32aa1n06x5               g234(.a(new_n309), .b(new_n326), .c(new_n304), .d(new_n317), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n298), .c(new_n193), .d(new_n294), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n330), .o1(new_n332));
  oao003aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .c(new_n324), .carry(new_n333));
  aoai13aa1n03x5               g238(.a(new_n333), .b(new_n332), .c(new_n307), .d(new_n295), .o1(new_n334));
  and002aa1n02x5               g239(.a(\b[29] ), .b(\a[30] ), .o(new_n335));
  oabi12aa1n02x5               g240(.a(new_n329), .b(\a[30] ), .c(\b[29] ), .out0(new_n336));
  oab012aa1n02x4               g241(.a(new_n336), .b(new_n324), .c(new_n335), .out0(new_n337));
  aoi022aa1n03x5               g242(.a(new_n334), .b(new_n329), .c(new_n331), .d(new_n337), .o1(\s[31] ));
  inv000aa1d42x5               g243(.a(\a[3] ), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n103), .b(\b[2] ), .c(new_n339), .out0(\s[3] ));
  orn002aa1n02x5               g245(.a(new_n99), .b(new_n103), .o(new_n341));
  xorc02aa1n02x5               g246(.a(\a[4] ), .b(\b[3] ), .out0(new_n342));
  aoib12aa1n02x5               g247(.a(new_n342), .b(new_n339), .c(\b[2] ), .out0(new_n343));
  aoi022aa1n02x5               g248(.a(new_n341), .b(new_n343), .c(new_n104), .d(new_n342), .o1(\s[4] ));
  and002aa1n02x5               g249(.a(\b[4] ), .b(\a[5] ), .o(new_n345));
  and002aa1n02x5               g250(.a(\b[3] ), .b(\a[4] ), .o(new_n346));
  nona32aa1n02x4               g251(.a(new_n104), .b(new_n113), .c(new_n346), .d(new_n345), .out0(new_n347));
  xnrc02aa1n02x5               g252(.a(\b[4] ), .b(\a[5] ), .out0(new_n348));
  nanb02aa1n02x5               g253(.a(new_n346), .b(new_n104), .out0(new_n349));
  aobi12aa1n02x5               g254(.a(new_n347), .b(new_n349), .c(new_n348), .out0(\s[5] ));
  norb02aa1n02x5               g255(.a(new_n112), .b(new_n111), .out0(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n351), .b(new_n347), .c(new_n119), .out0(\s[6] ));
  inv000aa1d42x5               g257(.a(new_n108), .o1(new_n353));
  norb02aa1n03x5               g258(.a(new_n351), .b(new_n347), .out0(new_n354));
  oai112aa1n02x5               g259(.a(new_n110), .b(new_n353), .c(new_n354), .d(new_n120), .o1(new_n355));
  aoi112aa1n02x5               g260(.a(new_n354), .b(new_n120), .c(new_n353), .d(new_n110), .o1(new_n356));
  norb02aa1n02x5               g261(.a(new_n355), .b(new_n356), .out0(\s[7] ));
  norb02aa1n02x5               g262(.a(new_n105), .b(new_n106), .out0(new_n358));
  xnbna2aa1n03x5               g263(.a(new_n358), .b(new_n355), .c(new_n353), .out0(\s[8] ));
  xnbna2aa1n03x5               g264(.a(new_n125), .b(new_n116), .c(new_n122), .out0(\s[9] ));
endmodule


