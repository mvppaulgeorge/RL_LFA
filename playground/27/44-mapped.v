// Benchmark "adder" written by ABC on Thu Jul 18 02:12:23 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n326, new_n328, new_n331, new_n332, new_n334,
    new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  oai112aa1n03x5               g008(.a(new_n103), .b(new_n101), .c(new_n102), .d(new_n100), .o1(new_n104));
  oa0022aa1n02x5               g009(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n105));
  aoi022aa1n06x5               g010(.a(new_n104), .b(new_n105), .c(\b[3] ), .d(\a[4] ), .o1(new_n106));
  norp02aa1n04x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nand42aa1n03x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norb02aa1n02x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nor042aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  norp02aa1n04x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  norb02aa1n02x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nano22aa1n03x7               g022(.a(new_n114), .b(new_n109), .c(new_n117), .out0(new_n118));
  oa0012aa1n02x5               g023(.a(new_n111), .b(new_n112), .c(new_n110), .o(new_n119));
  oai012aa1n02x5               g024(.a(new_n108), .b(new_n115), .c(new_n107), .o1(new_n120));
  oabi12aa1n06x5               g025(.a(new_n119), .b(new_n120), .c(new_n114), .out0(new_n121));
  xorc02aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n106), .d(new_n118), .o1(new_n123));
  nor002aa1n12x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n04x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norb02aa1d21x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g032(.a(new_n124), .o1(new_n128));
  inv000aa1d42x5               g033(.a(new_n126), .o1(new_n129));
  aoai13aa1n04x5               g034(.a(new_n128), .b(new_n129), .c(new_n123), .d(new_n99), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nor042aa1n04x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoi112aa1n02x5               g041(.a(new_n132), .b(new_n136), .c(new_n130), .d(new_n133), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n132), .c(new_n130), .d(new_n133), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(\s[12] ));
  nona23aa1n03x5               g044(.a(new_n135), .b(new_n133), .c(new_n132), .d(new_n134), .out0(new_n140));
  nano22aa1n03x7               g045(.a(new_n140), .b(new_n122), .c(new_n126), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n121), .c(new_n106), .d(new_n118), .o1(new_n142));
  nano23aa1n06x5               g047(.a(new_n132), .b(new_n134), .c(new_n135), .d(new_n133), .out0(new_n143));
  oai012aa1n03x5               g048(.a(new_n135), .b(new_n134), .c(new_n132), .o1(new_n144));
  aoai13aa1n12x5               g049(.a(new_n125), .b(new_n124), .c(new_n97), .d(new_n98), .o1(new_n145));
  inv000aa1n02x5               g050(.a(new_n145), .o1(new_n146));
  aobi12aa1n06x5               g051(.a(new_n144), .b(new_n143), .c(new_n146), .out0(new_n147));
  nor022aa1n06x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  xnbna2aa1n03x5               g055(.a(new_n150), .b(new_n142), .c(new_n147), .out0(\s[13] ));
  inv000aa1n06x5               g056(.a(new_n148), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[3] ), .b(\a[4] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n104), .b(new_n105), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(new_n154), .b(new_n153), .o1(new_n155));
  nano23aa1n03x5               g060(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n156));
  nanp03aa1n02x5               g061(.a(new_n156), .b(new_n109), .c(new_n117), .o1(new_n157));
  aoib12aa1n06x5               g062(.a(new_n119), .b(new_n156), .c(new_n120), .out0(new_n158));
  oai012aa1n03x5               g063(.a(new_n158), .b(new_n155), .c(new_n157), .o1(new_n159));
  tech160nm_fioai012aa1n05x5   g064(.a(new_n144), .b(new_n140), .c(new_n145), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n150), .b(new_n160), .c(new_n159), .d(new_n141), .o1(new_n161));
  nor002aa1n04x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  xnbna2aa1n03x5               g069(.a(new_n164), .b(new_n161), .c(new_n152), .out0(\s[14] ));
  nona23aa1n02x5               g070(.a(new_n163), .b(new_n149), .c(new_n148), .d(new_n162), .out0(new_n166));
  oaoi03aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n152), .o1(new_n167));
  inv000aa1n02x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n166), .c(new_n142), .d(new_n147), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nor042aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n06x4               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoi112aa1n02x5               g080(.a(new_n171), .b(new_n175), .c(new_n169), .d(new_n172), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n175), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  nanp02aa1n02x5               g083(.a(new_n106), .b(new_n118), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n172), .b(new_n171), .out0(new_n180));
  nano22aa1n03x7               g085(.a(new_n166), .b(new_n180), .c(new_n175), .out0(new_n181));
  nand02aa1n02x5               g086(.a(new_n141), .b(new_n181), .o1(new_n182));
  oai112aa1n02x5               g087(.a(new_n163), .b(new_n172), .c(new_n162), .d(new_n148), .o1(new_n183));
  nona22aa1n02x4               g088(.a(new_n183), .b(new_n173), .c(new_n171), .out0(new_n184));
  aoi022aa1d18x5               g089(.a(new_n160), .b(new_n181), .c(new_n174), .d(new_n184), .o1(new_n185));
  aoai13aa1n12x5               g090(.a(new_n185), .b(new_n182), .c(new_n179), .d(new_n158), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g092(.a(\a[18] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\a[17] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[16] ), .o1(new_n190));
  oaoi03aa1n03x5               g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  nano23aa1n02x4               g097(.a(new_n148), .b(new_n162), .c(new_n163), .d(new_n149), .out0(new_n193));
  nanp03aa1n02x5               g098(.a(new_n193), .b(new_n180), .c(new_n175), .o1(new_n194));
  nano32aa1n03x7               g099(.a(new_n194), .b(new_n143), .c(new_n126), .d(new_n122), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n121), .c(new_n106), .d(new_n118), .o1(new_n196));
  xroi22aa1d06x4               g101(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\b[17] ), .o1(new_n199));
  oaih22aa1d12x5               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  oa0012aa1n02x5               g105(.a(new_n200), .b(new_n199), .c(new_n188), .o(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n198), .c(new_n196), .d(new_n185), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nanp02aa1n04x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  xnrc02aa1n12x5               g112(.a(\b[19] ), .b(\a[20] ), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(new_n206), .b(new_n209), .c(new_n203), .d(new_n207), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n206), .o1(new_n211));
  nanb02aa1d24x5               g116(.a(new_n206), .b(new_n207), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n03x5               g118(.a(new_n213), .b(new_n201), .c(new_n186), .d(new_n197), .o1(new_n214));
  aoi012aa1n03x5               g119(.a(new_n208), .b(new_n214), .c(new_n211), .o1(new_n215));
  norp02aa1n03x5               g120(.a(new_n215), .b(new_n210), .o1(\s[20] ));
  nona22aa1n03x5               g121(.a(new_n197), .b(new_n212), .c(new_n208), .out0(new_n217));
  oai112aa1n02x5               g122(.a(new_n200), .b(new_n207), .c(new_n199), .d(new_n188), .o1(new_n218));
  oab012aa1n02x4               g123(.a(new_n206), .b(\a[20] ), .c(\b[19] ), .out0(new_n219));
  aoi022aa1n02x5               g124(.a(new_n218), .b(new_n219), .c(\b[19] ), .d(\a[20] ), .o1(new_n220));
  inv000aa1n02x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n04x5               g126(.a(new_n221), .b(new_n217), .c(new_n196), .d(new_n185), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nanp02aa1n04x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  nor002aa1n03x5               g131(.a(\b[21] ), .b(\a[22] ), .o1(new_n227));
  nand02aa1n03x5               g132(.a(\b[21] ), .b(\a[22] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n224), .b(new_n229), .c(new_n222), .d(new_n226), .o1(new_n230));
  inv000aa1n02x5               g135(.a(new_n224), .o1(new_n231));
  inv000aa1n02x5               g136(.a(new_n217), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n226), .b(new_n220), .c(new_n186), .d(new_n232), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n229), .o1(new_n234));
  aoi012aa1n03x5               g139(.a(new_n234), .b(new_n233), .c(new_n231), .o1(new_n235));
  norp02aa1n03x5               g140(.a(new_n235), .b(new_n230), .o1(\s[22] ));
  nona23aa1d18x5               g141(.a(new_n228), .b(new_n225), .c(new_n224), .d(new_n227), .out0(new_n237));
  nona32aa1n03x5               g142(.a(new_n197), .b(new_n237), .c(new_n208), .d(new_n212), .out0(new_n238));
  and002aa1n02x5               g143(.a(\b[19] ), .b(\a[20] ), .o(new_n239));
  aoi112aa1n02x5               g144(.a(new_n237), .b(new_n239), .c(new_n218), .d(new_n219), .o1(new_n240));
  oaoi03aa1n02x5               g145(.a(\a[22] ), .b(\b[21] ), .c(new_n231), .o1(new_n241));
  norp02aa1n02x5               g146(.a(new_n240), .b(new_n241), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n238), .c(new_n196), .d(new_n185), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n12x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(new_n247));
  xorc02aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  aoi112aa1n02x5               g153(.a(new_n245), .b(new_n248), .c(new_n243), .d(new_n247), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n245), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n238), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n242), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n247), .b(new_n252), .c(new_n186), .d(new_n251), .o1(new_n253));
  xnrc02aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .out0(new_n254));
  aoi012aa1n03x5               g159(.a(new_n254), .b(new_n253), .c(new_n250), .o1(new_n255));
  nor002aa1n02x5               g160(.a(new_n255), .b(new_n249), .o1(\s[24] ));
  inv000aa1d42x5               g161(.a(new_n237), .o1(new_n257));
  nano22aa1n03x7               g162(.a(new_n254), .b(new_n250), .c(new_n246), .out0(new_n258));
  nano22aa1n03x7               g163(.a(new_n217), .b(new_n257), .c(new_n258), .out0(new_n259));
  inv020aa1n03x5               g164(.a(new_n259), .o1(new_n260));
  nanp02aa1n03x5               g165(.a(new_n218), .b(new_n219), .o1(new_n261));
  nona23aa1n03x5               g166(.a(new_n261), .b(new_n258), .c(new_n237), .d(new_n239), .out0(new_n262));
  oaoi03aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .c(new_n250), .o1(new_n263));
  aoi013aa1n02x4               g168(.a(new_n263), .b(new_n241), .c(new_n248), .d(new_n247), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n262), .b(new_n264), .o1(new_n265));
  inv040aa1n03x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n04x5               g171(.a(new_n266), .b(new_n260), .c(new_n196), .d(new_n185), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  xorc02aa1n12x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  aoi112aa1n03x4               g176(.a(new_n269), .b(new_n271), .c(new_n267), .d(new_n270), .o1(new_n272));
  inv000aa1n02x5               g177(.a(new_n269), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n270), .b(new_n265), .c(new_n186), .d(new_n259), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n271), .o1(new_n275));
  aoi012aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n273), .o1(new_n276));
  nor002aa1n02x5               g181(.a(new_n276), .b(new_n272), .o1(\s[26] ));
  nanp02aa1n02x5               g182(.a(new_n184), .b(new_n174), .o1(new_n278));
  tech160nm_fioai012aa1n04x5   g183(.a(new_n278), .b(new_n147), .c(new_n194), .o1(new_n279));
  and002aa1n02x5               g184(.a(new_n271), .b(new_n270), .o(new_n280));
  nano22aa1n03x7               g185(.a(new_n238), .b(new_n258), .c(new_n280), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n279), .c(new_n159), .d(new_n195), .o1(new_n282));
  inv000aa1n02x5               g187(.a(new_n280), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .c(new_n273), .carry(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n283), .c(new_n262), .d(new_n264), .o1(new_n285));
  inv030aa1n02x5               g190(.a(new_n285), .o1(new_n286));
  nor042aa1n06x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  nanb02aa1n09x5               g193(.a(new_n287), .b(new_n288), .out0(new_n289));
  xobna2aa1n03x5               g194(.a(new_n289), .b(new_n282), .c(new_n286), .out0(\s[27] ));
  inv000aa1d42x5               g195(.a(new_n287), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n288), .b(new_n285), .c(new_n186), .d(new_n281), .o1(new_n292));
  tech160nm_fixnrc02aa1n05x5   g197(.a(\b[27] ), .b(\a[28] ), .out0(new_n293));
  aoi012aa1n03x5               g198(.a(new_n293), .b(new_n292), .c(new_n291), .o1(new_n294));
  aoi022aa1n02x7               g199(.a(new_n282), .b(new_n286), .c(\b[26] ), .d(\a[27] ), .o1(new_n295));
  nano22aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n293), .out0(new_n296));
  nor002aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[28] ));
  nor042aa1n02x5               g202(.a(new_n293), .b(new_n289), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n285), .c(new_n186), .d(new_n281), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .c(new_n291), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[28] ), .b(\a[29] ), .out0(new_n301));
  aoi012aa1n03x5               g206(.a(new_n301), .b(new_n299), .c(new_n300), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n298), .o1(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n303), .b(new_n282), .c(new_n286), .o1(new_n304));
  nano22aa1n03x5               g209(.a(new_n304), .b(new_n300), .c(new_n301), .out0(new_n305));
  nor002aa1n02x5               g210(.a(new_n302), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nor043aa1n03x5               g212(.a(new_n301), .b(new_n293), .c(new_n289), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n285), .c(new_n186), .d(new_n281), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[29] ), .b(\a[30] ), .out0(new_n311));
  aoi012aa1n03x5               g216(.a(new_n311), .b(new_n309), .c(new_n310), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n308), .o1(new_n313));
  tech160nm_fiaoi012aa1n03p5x5 g218(.a(new_n313), .b(new_n282), .c(new_n286), .o1(new_n314));
  nano22aa1n02x4               g219(.a(new_n314), .b(new_n310), .c(new_n311), .out0(new_n315));
  nor002aa1n02x5               g220(.a(new_n312), .b(new_n315), .o1(\s[30] ));
  xnrc02aa1n02x5               g221(.a(\b[30] ), .b(\a[31] ), .out0(new_n317));
  norb03aa1n03x5               g222(.a(new_n298), .b(new_n311), .c(new_n301), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n285), .c(new_n186), .d(new_n281), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .c(new_n310), .carry(new_n320));
  aoi012aa1n03x5               g225(.a(new_n317), .b(new_n319), .c(new_n320), .o1(new_n321));
  inv000aa1n02x5               g226(.a(new_n318), .o1(new_n322));
  tech160nm_fiaoi012aa1n03p5x5 g227(.a(new_n322), .b(new_n282), .c(new_n286), .o1(new_n323));
  nano22aa1n03x5               g228(.a(new_n323), .b(new_n317), .c(new_n320), .out0(new_n324));
  nor002aa1n02x5               g229(.a(new_n321), .b(new_n324), .o1(\s[31] ));
  oai012aa1n02x5               g230(.a(new_n101), .b(new_n102), .c(new_n100), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g232(.a(\a[3] ), .b(\b[2] ), .c(new_n326), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g234(.a(new_n106), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi13aa1n02x5               g235(.a(new_n109), .b(new_n116), .c(new_n106), .d(new_n115), .o1(new_n331));
  oai112aa1n02x5               g236(.a(new_n116), .b(new_n109), .c(new_n106), .d(new_n115), .o1(new_n332));
  norb02aa1n02x5               g237(.a(new_n332), .b(new_n331), .out0(\s[6] ));
  norb02aa1n03x4               g238(.a(new_n332), .b(new_n107), .out0(new_n334));
  xnrb03aa1n02x5               g239(.a(new_n334), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g240(.a(\a[7] ), .b(\b[6] ), .c(new_n334), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g242(.a(new_n122), .b(new_n179), .c(new_n158), .out0(\s[9] ));
endmodule


