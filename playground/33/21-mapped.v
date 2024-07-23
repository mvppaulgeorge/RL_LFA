// Benchmark "adder" written by ABC on Thu Jul 18 05:02:32 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n298, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n321, new_n322, new_n325, new_n326,
    new_n327, new_n329, new_n331, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor002aa1n03x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nand42aa1n02x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nanb02aa1n06x5               g004(.a(new_n98), .b(new_n99), .out0(new_n100));
  oai112aa1n06x5               g005(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  oai022aa1n02x5               g008(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n104));
  oabi12aa1n06x5               g009(.a(new_n104), .b(new_n103), .c(new_n100), .out0(new_n105));
  nanp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  orn002aa1n02x5               g011(.a(\a[6] ), .b(\b[5] ), .o(new_n107));
  nand42aa1n03x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  tech160nm_fixnrc02aa1n04x5   g013(.a(\b[4] ), .b(\a[5] ), .out0(new_n109));
  nano32aa1n03x7               g014(.a(new_n109), .b(new_n107), .c(new_n108), .d(new_n106), .out0(new_n110));
  nanp02aa1n12x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n03x7               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  nand23aa1n02x5               g020(.a(new_n105), .b(new_n110), .c(new_n115), .o1(new_n116));
  nona22aa1n02x4               g021(.a(new_n111), .b(new_n112), .c(new_n113), .out0(new_n117));
  nor042aa1n06x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  aob012aa1n03x5               g023(.a(new_n107), .b(new_n118), .c(new_n106), .out0(new_n119));
  aoi022aa1n09x5               g024(.a(new_n115), .b(new_n119), .c(new_n111), .d(new_n117), .o1(new_n120));
  oai112aa1n02x5               g025(.a(new_n116), .b(new_n120), .c(\b[8] ), .d(\a[9] ), .o1(new_n121));
  nor042aa1n06x5               g026(.a(\b[9] ), .b(\a[10] ), .o1(new_n122));
  nanp02aa1n04x5               g027(.a(\b[9] ), .b(\a[10] ), .o1(new_n123));
  nanb02aa1n02x5               g028(.a(new_n122), .b(new_n123), .out0(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n124), .b(new_n121), .c(new_n97), .out0(\s[10] ));
  nand22aa1n03x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nor002aa1n02x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nanb02aa1n03x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n123), .b(new_n122), .c(new_n121), .d(new_n97), .o1(new_n129));
  nano22aa1n02x4               g034(.a(new_n127), .b(new_n123), .c(new_n126), .out0(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n122), .c(new_n121), .d(new_n97), .o1(new_n131));
  aobi12aa1n02x5               g036(.a(new_n131), .b(new_n129), .c(new_n128), .out0(\s[11] ));
  oai012aa1n02x5               g037(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .o1(new_n133));
  xorc02aa1n12x5               g038(.a(\a[12] ), .b(\b[11] ), .out0(new_n134));
  norp02aa1n02x5               g039(.a(new_n134), .b(new_n127), .o1(new_n135));
  aoi022aa1n02x5               g040(.a(new_n133), .b(new_n134), .c(new_n131), .d(new_n135), .o1(\s[12] ));
  oab012aa1n02x5               g041(.a(new_n104), .b(new_n103), .c(new_n100), .out0(new_n137));
  nand02aa1n02x5               g042(.a(new_n110), .b(new_n115), .o1(new_n138));
  oai012aa1n12x5               g043(.a(new_n120), .b(new_n138), .c(new_n137), .o1(new_n139));
  nona23aa1n06x5               g044(.a(new_n126), .b(new_n123), .c(new_n127), .d(new_n122), .out0(new_n140));
  xnrc02aa1n12x5               g045(.a(\b[8] ), .b(\a[9] ), .out0(new_n141));
  norb03aa1n06x5               g046(.a(new_n134), .b(new_n140), .c(new_n141), .out0(new_n142));
  oai022aa1n02x5               g047(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n143));
  nanp03aa1n02x5               g048(.a(new_n143), .b(new_n123), .c(new_n126), .o1(new_n144));
  oai122aa1n06x5               g049(.a(new_n144), .b(\a[12] ), .c(\b[11] ), .d(\a[11] ), .e(\b[10] ), .o1(new_n145));
  aobi12aa1n02x5               g050(.a(new_n145), .b(\b[11] ), .c(\a[12] ), .out0(new_n146));
  xorc02aa1n02x5               g051(.a(\a[13] ), .b(\b[12] ), .out0(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n146), .c(new_n139), .d(new_n142), .o1(new_n148));
  aoi112aa1n02x5               g053(.a(new_n147), .b(new_n146), .c(new_n139), .d(new_n142), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n148), .b(new_n149), .out0(\s[13] ));
  inv000aa1d42x5               g055(.a(\a[13] ), .o1(new_n151));
  inv000aa1d42x5               g056(.a(\b[12] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n152), .b(new_n151), .o1(new_n153));
  xorc02aa1n02x5               g058(.a(\a[14] ), .b(\b[13] ), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n153), .out0(\s[14] ));
  inv000aa1d42x5               g060(.a(\a[14] ), .o1(new_n156));
  xroi22aa1d04x5               g061(.a(new_n151), .b(\b[12] ), .c(new_n156), .d(\b[13] ), .out0(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n146), .c(new_n139), .d(new_n142), .o1(new_n158));
  aoi112aa1n03x5               g063(.a(\b[12] ), .b(\a[13] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n159));
  aoib12aa1n02x7               g064(.a(new_n159), .b(new_n156), .c(\b[13] ), .out0(new_n160));
  nor002aa1d32x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nanp02aa1n04x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n158), .c(new_n160), .out0(\s[15] ));
  aob012aa1n03x5               g069(.a(new_n163), .b(new_n158), .c(new_n160), .out0(new_n165));
  nor002aa1d24x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nand42aa1n04x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n166), .o1(new_n169));
  aoi012aa1n02x5               g074(.a(new_n161), .b(new_n169), .c(new_n167), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n161), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n163), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n172), .c(new_n158), .d(new_n160), .o1(new_n173));
  aoi022aa1n03x5               g078(.a(new_n173), .b(new_n168), .c(new_n165), .d(new_n170), .o1(\s[16] ));
  nona32aa1n03x5               g079(.a(new_n134), .b(new_n141), .c(new_n128), .d(new_n124), .out0(new_n175));
  nona23aa1n09x5               g080(.a(new_n167), .b(new_n162), .c(new_n161), .d(new_n166), .out0(new_n176));
  inv000aa1n02x5               g081(.a(new_n176), .o1(new_n177));
  nano22aa1n03x7               g082(.a(new_n175), .b(new_n157), .c(new_n177), .out0(new_n178));
  nanp02aa1n02x5               g083(.a(new_n139), .b(new_n178), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n134), .b(new_n141), .out0(new_n180));
  nona23aa1n09x5               g085(.a(new_n180), .b(new_n157), .c(new_n176), .d(new_n140), .out0(new_n181));
  nanb03aa1n03x5               g086(.a(new_n166), .b(new_n167), .c(new_n162), .out0(new_n182));
  aoi022aa1n02x5               g087(.a(new_n152), .b(new_n151), .c(\a[12] ), .d(\b[11] ), .o1(new_n183));
  oai022aa1n02x5               g088(.a(new_n151), .b(new_n152), .c(\b[13] ), .d(\a[14] ), .o1(new_n184));
  tech160nm_fiaoi012aa1n03p5x5 g089(.a(new_n161), .b(\a[14] ), .c(\b[13] ), .o1(new_n185));
  nano23aa1n03x7               g090(.a(new_n182), .b(new_n184), .c(new_n183), .d(new_n185), .out0(new_n186));
  aob012aa1n02x5               g091(.a(new_n169), .b(new_n161), .c(new_n167), .out0(new_n187));
  oabi12aa1n06x5               g092(.a(new_n187), .b(new_n176), .c(new_n160), .out0(new_n188));
  aoi012aa1n06x5               g093(.a(new_n188), .b(new_n186), .c(new_n145), .o1(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n181), .c(new_n116), .d(new_n120), .o1(new_n190));
  xorc02aa1n12x5               g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  aoi112aa1n02x5               g096(.a(new_n188), .b(new_n191), .c(new_n186), .d(new_n145), .o1(new_n192));
  aoi022aa1n02x5               g097(.a(new_n190), .b(new_n191), .c(new_n179), .d(new_n192), .o1(\s[17] ));
  inv020aa1n10x5               g098(.a(\a[17] ), .o1(new_n194));
  nanb02aa1n12x5               g099(.a(\b[16] ), .b(new_n194), .out0(new_n195));
  inv020aa1n02x5               g100(.a(new_n189), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n191), .b(new_n196), .c(new_n139), .d(new_n178), .o1(new_n197));
  xorc02aa1n12x5               g102(.a(\a[18] ), .b(\b[17] ), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n197), .c(new_n195), .out0(\s[18] ));
  and002aa1n02x5               g104(.a(new_n198), .b(new_n191), .o(new_n200));
  aoai13aa1n02x5               g105(.a(new_n200), .b(new_n196), .c(new_n139), .d(new_n178), .o1(new_n201));
  oaoi03aa1n12x5               g106(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  nor042aa1n09x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand02aa1n06x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n201), .c(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g113(.a(new_n206), .b(new_n202), .c(new_n190), .d(new_n200), .o1(new_n209));
  nor002aa1n03x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand02aa1n08x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  aoib12aa1n02x5               g117(.a(new_n204), .b(new_n211), .c(new_n210), .out0(new_n213));
  inv000aa1n02x5               g118(.a(new_n204), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(new_n209), .b(new_n214), .o1(new_n215));
  aoi022aa1n02x5               g120(.a(new_n215), .b(new_n212), .c(new_n209), .d(new_n213), .o1(\s[20] ));
  nano23aa1n06x5               g121(.a(new_n204), .b(new_n210), .c(new_n211), .d(new_n205), .out0(new_n217));
  nand23aa1n06x5               g122(.a(new_n217), .b(new_n191), .c(new_n198), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  oaoi03aa1n02x5               g124(.a(\a[20] ), .b(\b[19] ), .c(new_n214), .o1(new_n220));
  aoi012aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n202), .o1(new_n221));
  inv000aa1n02x5               g126(.a(new_n221), .o1(new_n222));
  xnrc02aa1n12x5               g127(.a(\b[20] ), .b(\a[21] ), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n222), .c(new_n190), .d(new_n219), .o1(new_n225));
  aoi112aa1n02x5               g130(.a(new_n224), .b(new_n222), .c(new_n190), .d(new_n219), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n225), .b(new_n226), .out0(\s[21] ));
  nor002aa1n04x5               g132(.a(\b[21] ), .b(\a[22] ), .o1(new_n228));
  nand02aa1n08x5               g133(.a(\b[21] ), .b(\a[22] ), .o1(new_n229));
  nanb02aa1d30x5               g134(.a(new_n228), .b(new_n229), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  norp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  aoib12aa1n02x5               g137(.a(new_n232), .b(new_n229), .c(new_n228), .out0(new_n233));
  tech160nm_fioai012aa1n03p5x5 g138(.a(new_n225), .b(\b[20] ), .c(\a[21] ), .o1(new_n234));
  aoi022aa1n02x7               g139(.a(new_n234), .b(new_n231), .c(new_n225), .d(new_n233), .o1(\s[22] ));
  nor022aa1n16x5               g140(.a(new_n223), .b(new_n230), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nano32aa1n02x4               g142(.a(new_n237), .b(new_n217), .c(new_n198), .d(new_n191), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n236), .b(new_n220), .c(new_n217), .d(new_n202), .o1(new_n239));
  oai012aa1n02x5               g144(.a(new_n229), .b(new_n228), .c(new_n232), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(new_n239), .b(new_n240), .o1(new_n241));
  xorc02aa1n12x5               g146(.a(\a[23] ), .b(\b[22] ), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n241), .c(new_n190), .d(new_n238), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(new_n242), .b(new_n241), .c(new_n190), .d(new_n238), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n243), .b(new_n244), .out0(\s[23] ));
  tech160nm_fixorc02aa1n05x5   g150(.a(\a[24] ), .b(\b[23] ), .out0(new_n246));
  nor042aa1d18x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  norp02aa1n02x5               g152(.a(new_n246), .b(new_n247), .o1(new_n248));
  inv000aa1n09x5               g153(.a(new_n247), .o1(new_n249));
  nand42aa1n02x5               g154(.a(new_n243), .b(new_n249), .o1(new_n250));
  aoi022aa1n02x7               g155(.a(new_n250), .b(new_n246), .c(new_n243), .d(new_n248), .o1(\s[24] ));
  and002aa1n06x5               g156(.a(new_n246), .b(new_n242), .o(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  tech160nm_fioaoi03aa1n02p5x5 g158(.a(\a[24] ), .b(\b[23] ), .c(new_n249), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n253), .c(new_n239), .d(new_n240), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  nona32aa1n06x5               g162(.a(new_n190), .b(new_n253), .c(new_n237), .d(new_n218), .out0(new_n258));
  xorc02aa1n12x5               g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n258), .c(new_n257), .out0(\s[25] ));
  nand42aa1n02x5               g165(.a(new_n258), .b(new_n257), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(new_n261), .b(new_n259), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  orn002aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .o(new_n264));
  norb02aa1n02x5               g169(.a(new_n264), .b(new_n263), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n259), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n264), .b(new_n266), .c(new_n258), .d(new_n257), .o1(new_n267));
  aoi022aa1n03x5               g172(.a(new_n267), .b(new_n263), .c(new_n262), .d(new_n265), .o1(\s[26] ));
  and002aa1n12x5               g173(.a(new_n263), .b(new_n259), .o(new_n269));
  nano23aa1n06x5               g174(.a(new_n218), .b(new_n253), .c(new_n269), .d(new_n236), .out0(new_n270));
  aoai13aa1n12x5               g175(.a(new_n270), .b(new_n196), .c(new_n139), .d(new_n178), .o1(new_n271));
  nanp02aa1n03x5               g176(.a(new_n256), .b(new_n269), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(\b[25] ), .b(\a[26] ), .o1(new_n273));
  oai022aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(new_n274), .b(new_n273), .o1(new_n275));
  nand23aa1n06x5               g180(.a(new_n271), .b(new_n272), .c(new_n275), .o1(new_n276));
  tech160nm_fixorc02aa1n04x5   g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  aoi122aa1n02x7               g182(.a(new_n277), .b(new_n273), .c(new_n274), .d(new_n256), .e(new_n269), .o1(new_n278));
  aoi022aa1n03x5               g183(.a(new_n278), .b(new_n271), .c(new_n276), .d(new_n277), .o1(\s[27] ));
  nanp02aa1n03x5               g184(.a(new_n276), .b(new_n277), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .out0(new_n281));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n281), .b(new_n282), .o1(new_n283));
  aoi022aa1n03x5               g188(.a(new_n256), .b(new_n269), .c(new_n273), .d(new_n274), .o1(new_n284));
  inv000aa1n06x5               g189(.a(new_n282), .o1(new_n285));
  inv040aa1n03x5               g190(.a(new_n277), .o1(new_n286));
  aoai13aa1n02x7               g191(.a(new_n285), .b(new_n286), .c(new_n284), .d(new_n271), .o1(new_n287));
  aoi022aa1n03x5               g192(.a(new_n287), .b(new_n281), .c(new_n280), .d(new_n283), .o1(\s[28] ));
  and002aa1n02x5               g193(.a(new_n281), .b(new_n277), .o(new_n289));
  nand22aa1n03x5               g194(.a(new_n276), .b(new_n289), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .out0(new_n291));
  oaoi03aa1n12x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n285), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n292), .b(new_n291), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n289), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n292), .o1(new_n295));
  aoai13aa1n02x7               g200(.a(new_n295), .b(new_n294), .c(new_n284), .d(new_n271), .o1(new_n296));
  aoi022aa1n03x5               g201(.a(new_n296), .b(new_n291), .c(new_n290), .d(new_n293), .o1(\s[29] ));
  nanp02aa1n02x5               g202(.a(\b[0] ), .b(\a[1] ), .o1(new_n298));
  xorb03aa1n02x5               g203(.a(new_n298), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g204(.a(new_n286), .b(new_n281), .c(new_n291), .out0(new_n300));
  nanp02aa1n03x5               g205(.a(new_n276), .b(new_n300), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .out0(new_n302));
  inv000aa1d42x5               g207(.a(\a[29] ), .o1(new_n303));
  inv000aa1d42x5               g208(.a(\b[28] ), .o1(new_n304));
  oabi12aa1n02x5               g209(.a(new_n302), .b(\a[29] ), .c(\b[28] ), .out0(new_n305));
  oaoi13aa1n02x5               g210(.a(new_n305), .b(new_n292), .c(new_n303), .d(new_n304), .o1(new_n306));
  inv000aa1n02x5               g211(.a(new_n300), .o1(new_n307));
  oaoi03aa1n02x5               g212(.a(new_n303), .b(new_n304), .c(new_n292), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n307), .c(new_n284), .d(new_n271), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n309), .b(new_n302), .c(new_n301), .d(new_n306), .o1(\s[30] ));
  nano32aa1n02x4               g215(.a(new_n286), .b(new_n302), .c(new_n281), .d(new_n291), .out0(new_n311));
  nanp02aa1n03x5               g216(.a(new_n276), .b(new_n311), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[31] ), .b(\b[30] ), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .carry(new_n314));
  norb02aa1n02x5               g219(.a(new_n314), .b(new_n313), .out0(new_n315));
  inv000aa1n02x5               g220(.a(new_n311), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n314), .b(new_n316), .c(new_n284), .d(new_n271), .o1(new_n317));
  aoi022aa1n03x5               g222(.a(new_n317), .b(new_n313), .c(new_n312), .d(new_n315), .o1(\s[31] ));
  xnbna2aa1n03x5               g223(.a(new_n100), .b(new_n101), .c(new_n102), .out0(\s[3] ));
  orn002aa1n02x5               g224(.a(\a[4] ), .b(\b[3] ), .o(new_n320));
  xorc02aa1n02x5               g225(.a(\a[4] ), .b(\b[3] ), .out0(new_n321));
  aoi113aa1n02x5               g226(.a(new_n321), .b(new_n98), .c(new_n101), .d(new_n102), .e(new_n99), .o1(new_n322));
  aoi013aa1n02x4               g227(.a(new_n322), .b(new_n105), .c(new_n108), .d(new_n320), .o1(\s[4] ));
  xnbna2aa1n03x5               g228(.a(new_n109), .b(new_n105), .c(new_n108), .out0(\s[5] ));
  xorc02aa1n02x5               g229(.a(\a[6] ), .b(\b[5] ), .out0(new_n325));
  inv000aa1d42x5               g230(.a(new_n118), .o1(new_n326));
  nanb03aa1n02x5               g231(.a(new_n109), .b(new_n105), .c(new_n108), .out0(new_n327));
  xnbna2aa1n03x5               g232(.a(new_n325), .b(new_n327), .c(new_n326), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g233(.a(new_n119), .b(new_n105), .c(new_n110), .o(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norb02aa1n02x5               g235(.a(new_n111), .b(new_n112), .out0(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n113), .c(new_n329), .d(new_n114), .o1(new_n332));
  aoi112aa1n02x5               g237(.a(new_n331), .b(new_n113), .c(new_n329), .d(new_n114), .o1(new_n333));
  norb02aa1n02x5               g238(.a(new_n332), .b(new_n333), .out0(\s[8] ));
  xobna2aa1n03x5               g239(.a(new_n141), .b(new_n116), .c(new_n120), .out0(\s[9] ));
endmodule

