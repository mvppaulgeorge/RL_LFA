// Benchmark "adder" written by ABC on Thu Jul 18 09:40:34 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n304, new_n305, new_n307, new_n310, new_n312, new_n313,
    new_n315, new_n316, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixnrc02aa1n02p5x5 g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor002aa1n04x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  and002aa1n06x5               g004(.a(\b[6] ), .b(\a[7] ), .o(new_n100));
  inv030aa1n03x5               g005(.a(new_n100), .o1(new_n101));
  and002aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o(new_n102));
  oa0022aa1n06x5               g007(.a(\a[8] ), .b(\b[7] ), .c(\a[7] ), .d(\b[6] ), .o(new_n103));
  nor022aa1n04x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nand42aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nor022aa1n06x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nand22aa1n04x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nona23aa1n03x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  nano23aa1n06x5               g013(.a(new_n108), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n109));
  and002aa1n06x5               g014(.a(\b[3] ), .b(\a[4] ), .o(new_n110));
  nor022aa1n16x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  oab012aa1n02x4               g016(.a(new_n111), .b(\a[4] ), .c(\b[3] ), .out0(new_n112));
  nand42aa1n08x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  nand42aa1n04x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanb03aa1n09x5               g019(.a(new_n111), .b(new_n114), .c(new_n113), .out0(new_n115));
  nand02aa1d04x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  nor042aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  norb03aa1n09x5               g022(.a(new_n113), .b(new_n117), .c(new_n116), .out0(new_n118));
  oaoi13aa1n12x5               g023(.a(new_n110), .b(new_n112), .c(new_n118), .d(new_n115), .o1(new_n119));
  aoai13aa1n02x5               g024(.a(new_n101), .b(new_n106), .c(new_n104), .d(new_n107), .o1(new_n120));
  aoi022aa1n09x5               g025(.a(new_n120), .b(new_n103), .c(\a[8] ), .d(\b[7] ), .o1(new_n121));
  aoai13aa1n03x5               g026(.a(new_n99), .b(new_n121), .c(new_n119), .d(new_n109), .o1(new_n122));
  nona22aa1n03x5               g027(.a(new_n122), .b(new_n98), .c(new_n97), .out0(new_n123));
  nano23aa1n03x5               g028(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n124));
  nona23aa1n02x4               g029(.a(new_n124), .b(new_n103), .c(new_n100), .d(new_n102), .out0(new_n125));
  inv000aa1d42x5               g030(.a(new_n110), .o1(new_n126));
  oai012aa1n02x5               g031(.a(new_n112), .b(new_n118), .c(new_n115), .o1(new_n127));
  nand02aa1n02x5               g032(.a(new_n127), .b(new_n126), .o1(new_n128));
  oabi12aa1n03x5               g033(.a(new_n121), .b(new_n128), .c(new_n125), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n97), .b(new_n98), .c(new_n129), .d(new_n99), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n130), .b(new_n123), .o1(\s[10] ));
  inv000aa1d42x5               g036(.a(\a[10] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(\b[9] ), .o1(new_n133));
  nor002aa1n16x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n16x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  oaoi13aa1n02x5               g041(.a(new_n136), .b(new_n123), .c(new_n132), .d(new_n133), .o1(new_n137));
  oai112aa1n03x5               g042(.a(new_n123), .b(new_n136), .c(new_n133), .d(new_n132), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(new_n134), .o1(new_n140));
  nor002aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  xobna2aa1n03x5               g048(.a(new_n143), .b(new_n138), .c(new_n140), .out0(\s[12] ));
  nano23aa1d15x5               g049(.a(new_n134), .b(new_n141), .c(new_n142), .d(new_n135), .out0(new_n145));
  nanb02aa1n03x5               g050(.a(new_n98), .b(new_n99), .out0(new_n146));
  nona22aa1n12x5               g051(.a(new_n145), .b(new_n97), .c(new_n146), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n04x5               g053(.a(new_n148), .b(new_n121), .c(new_n119), .d(new_n109), .o1(new_n149));
  oao003aa1n02x5               g054(.a(new_n132), .b(new_n133), .c(new_n98), .carry(new_n150));
  oaoi03aa1n02x5               g055(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n151));
  aoi012aa1n06x5               g056(.a(new_n151), .b(new_n145), .c(new_n150), .o1(new_n152));
  xnrc02aa1n12x5               g057(.a(\b[12] ), .b(\a[13] ), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n149), .c(new_n152), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(\a[14] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(new_n149), .b(new_n152), .o1(new_n157));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n158), .b(new_n157), .c(new_n154), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(new_n156), .out0(\s[14] ));
  xnrc02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .out0(new_n161));
  norp02aa1n02x5               g066(.a(new_n161), .b(new_n153), .o1(new_n162));
  inv000aa1n02x5               g067(.a(new_n162), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\b[13] ), .o1(new_n164));
  oaoi03aa1n02x5               g069(.a(new_n156), .b(new_n164), .c(new_n158), .o1(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n163), .c(new_n149), .d(new_n152), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  tech160nm_finand02aa1n03p5x5 g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nor002aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoi112aa1n02x5               g077(.a(new_n168), .b(new_n172), .c(new_n166), .d(new_n169), .o1(new_n173));
  aoai13aa1n03x5               g078(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n174));
  norb02aa1n02x7               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  nano23aa1n03x5               g080(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n176));
  nona22aa1n03x5               g081(.a(new_n176), .b(new_n161), .c(new_n153), .out0(new_n177));
  nor042aa1n06x5               g082(.a(new_n177), .b(new_n147), .o1(new_n178));
  aoai13aa1n09x5               g083(.a(new_n178), .b(new_n121), .c(new_n119), .d(new_n109), .o1(new_n179));
  oai012aa1n02x5               g084(.a(new_n171), .b(new_n170), .c(new_n168), .o1(new_n180));
  oaib12aa1n09x5               g085(.a(new_n180), .b(new_n165), .c(new_n176), .out0(new_n181));
  oab012aa1n09x5               g086(.a(new_n181), .b(new_n152), .c(new_n177), .out0(new_n182));
  nand02aa1d08x5               g087(.a(new_n179), .b(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  oai022aa1d24x5               g095(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n191));
  oaib12aa1n18x5               g096(.a(new_n191), .b(new_n185), .c(\b[17] ), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  norp02aa1n03x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nand42aa1n06x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand42aa1n03x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n02x5               g108(.a(new_n197), .b(new_n203), .c(new_n194), .out0(new_n204));
  orn002aa1n24x5               g109(.a(\a[19] ), .b(\b[18] ), .o(new_n205));
  aobi12aa1n06x5               g110(.a(new_n203), .b(new_n197), .c(new_n205), .out0(new_n206));
  norb02aa1n03x4               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n03x5               g112(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n190), .b(new_n208), .o1(new_n209));
  nona23aa1n06x5               g114(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n210));
  oaoi03aa1n06x5               g115(.a(\a[20] ), .b(\b[19] ), .c(new_n205), .o1(new_n211));
  inv000aa1n02x5               g116(.a(new_n211), .o1(new_n212));
  oai012aa1n18x5               g117(.a(new_n212), .b(new_n210), .c(new_n192), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n209), .c(new_n179), .d(new_n182), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n02x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoi112aa1n02x5               g124(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(\s[22] ));
  inv000aa1d42x5               g127(.a(\a[21] ), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\a[22] ), .o1(new_n224));
  xroi22aa1d04x5               g129(.a(new_n223), .b(\b[20] ), .c(new_n224), .d(\b[21] ), .out0(new_n225));
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n190), .c(new_n208), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  oao003aa1n02x5               g132(.a(new_n224), .b(new_n227), .c(new_n217), .carry(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n213), .c(new_n225), .o1(new_n229));
  aoai13aa1n04x5               g134(.a(new_n229), .b(new_n226), .c(new_n179), .d(new_n182), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x7               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  aoai13aa1n03x5               g142(.a(new_n225), .b(new_n211), .c(new_n208), .d(new_n193), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n228), .o1(new_n239));
  and002aa1n06x5               g144(.a(new_n234), .b(new_n233), .o(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  oai022aa1n02x5               g146(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n242));
  aob012aa1n02x5               g147(.a(new_n242), .b(\b[23] ), .c(\a[24] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n241), .c(new_n238), .d(new_n239), .o1(new_n244));
  nano32aa1n02x4               g149(.a(new_n241), .b(new_n225), .c(new_n190), .d(new_n208), .out0(new_n245));
  xorc02aa1n02x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n244), .c(new_n183), .d(new_n245), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n244), .b(new_n246), .c(new_n183), .d(new_n245), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[25] ));
  nor042aa1n03x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  nona22aa1n02x5               g156(.a(new_n247), .b(new_n251), .c(new_n250), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n250), .o1(new_n253));
  aobi12aa1n06x5               g158(.a(new_n251), .b(new_n247), .c(new_n253), .out0(new_n254));
  norb02aa1n03x4               g159(.a(new_n252), .b(new_n254), .out0(\s[26] ));
  oabi12aa1n02x5               g160(.a(new_n181), .b(new_n152), .c(new_n177), .out0(new_n256));
  inv000aa1d42x5               g161(.a(\a[25] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(\a[26] ), .o1(new_n258));
  xroi22aa1d06x4               g163(.a(new_n257), .b(\b[24] ), .c(new_n258), .d(\b[25] ), .out0(new_n259));
  nano22aa1n03x7               g164(.a(new_n226), .b(new_n240), .c(new_n259), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n256), .c(new_n129), .d(new_n178), .o1(new_n261));
  oao003aa1n03x5               g166(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n262));
  aobi12aa1n06x5               g167(.a(new_n262), .b(new_n244), .c(new_n259), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n261), .out0(\s[27] ));
  norp02aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  inv040aa1n03x5               g171(.a(new_n266), .o1(new_n267));
  aobi12aa1n02x5               g172(.a(new_n264), .b(new_n263), .c(new_n261), .out0(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[27] ), .b(\a[28] ), .out0(new_n269));
  nano22aa1n02x4               g174(.a(new_n268), .b(new_n267), .c(new_n269), .out0(new_n270));
  inv000aa1n02x5               g175(.a(new_n260), .o1(new_n271));
  tech160nm_fiaoi012aa1n05x5   g176(.a(new_n271), .b(new_n179), .c(new_n182), .o1(new_n272));
  aoai13aa1n04x5               g177(.a(new_n240), .b(new_n228), .c(new_n213), .d(new_n225), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n259), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n262), .b(new_n274), .c(new_n273), .d(new_n243), .o1(new_n275));
  oaih12aa1n02x5               g180(.a(new_n264), .b(new_n275), .c(new_n272), .o1(new_n276));
  tech160nm_fiaoi012aa1n02p5x5 g181(.a(new_n269), .b(new_n276), .c(new_n267), .o1(new_n277));
  norp02aa1n03x5               g182(.a(new_n277), .b(new_n270), .o1(\s[28] ));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  norb02aa1n02x5               g184(.a(new_n264), .b(new_n269), .out0(new_n280));
  aobi12aa1n02x5               g185(.a(new_n280), .b(new_n263), .c(new_n261), .out0(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n267), .carry(new_n282));
  nano22aa1n02x4               g187(.a(new_n281), .b(new_n279), .c(new_n282), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n280), .b(new_n275), .c(new_n272), .o1(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n279), .b(new_n284), .c(new_n282), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n285), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g192(.a(new_n264), .b(new_n279), .c(new_n269), .out0(new_n288));
  aobi12aa1n02x7               g193(.a(new_n288), .b(new_n263), .c(new_n261), .out0(new_n289));
  oao003aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n289), .b(new_n290), .c(new_n291), .out0(new_n292));
  oaih12aa1n02x5               g197(.a(new_n288), .b(new_n275), .c(new_n272), .o1(new_n293));
  tech160nm_fiaoi012aa1n02p5x5 g198(.a(new_n291), .b(new_n293), .c(new_n290), .o1(new_n294));
  norp02aa1n03x5               g199(.a(new_n294), .b(new_n292), .o1(\s[30] ));
  norb02aa1n02x5               g200(.a(new_n288), .b(new_n291), .out0(new_n296));
  aobi12aa1n02x7               g201(.a(new_n296), .b(new_n263), .c(new_n261), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[30] ), .b(\b[29] ), .c(new_n290), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[30] ), .b(\a[31] ), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  oaih12aa1n02x5               g205(.a(new_n296), .b(new_n275), .c(new_n272), .o1(new_n301));
  aoi012aa1n02x7               g206(.a(new_n299), .b(new_n301), .c(new_n298), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n300), .o1(\s[31] ));
  norb02aa1n02x5               g208(.a(new_n114), .b(new_n111), .out0(new_n304));
  oaoi13aa1n02x5               g209(.a(new_n304), .b(new_n113), .c(new_n117), .d(new_n116), .o1(new_n305));
  oab012aa1n02x4               g210(.a(new_n305), .b(new_n115), .c(new_n118), .out0(\s[3] ));
  oabi12aa1n02x5               g211(.a(new_n111), .b(new_n118), .c(new_n115), .out0(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi013aa1n02x4               g214(.a(new_n104), .b(new_n127), .c(new_n126), .d(new_n105), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb03aa1n02x5               g216(.a(new_n106), .b(new_n310), .c(new_n107), .out0(new_n312));
  xorc02aa1n02x5               g217(.a(\a[7] ), .b(\b[6] ), .out0(new_n313));
  xobna2aa1n03x5               g218(.a(new_n313), .b(new_n312), .c(new_n107), .out0(\s[7] ));
  orn002aa1n02x5               g219(.a(\a[7] ), .b(\b[6] ), .o(new_n315));
  nanp03aa1n02x5               g220(.a(new_n312), .b(new_n107), .c(new_n313), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[8] ), .b(\b[7] ), .out0(new_n317));
  xnbna2aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n315), .out0(\s[8] ));
  xorb03aa1n02x5               g223(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


