// Benchmark "adder" written by ABC on Thu Jul 18 05:00:41 2024

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
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n327, new_n328, new_n330, new_n331,
    new_n334, new_n336, new_n337, new_n338, new_n339, new_n341, new_n343;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n03x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand22aa1n03x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  oab012aa1n06x5               g003(.a(new_n98), .b(\a[2] ), .c(\b[1] ), .out0(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  inv040aa1d32x5               g005(.a(\a[3] ), .o1(new_n101));
  inv030aa1d32x5               g006(.a(\b[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand23aa1n03x5               g009(.a(new_n103), .b(new_n100), .c(new_n104), .o1(new_n105));
  oa0022aa1n02x5               g010(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n106));
  tech160nm_fioai012aa1n04x5   g011(.a(new_n106), .b(new_n105), .c(new_n99), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\a[7] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[6] ), .o1(new_n110));
  aoi022aa1n06x5               g015(.a(new_n110), .b(new_n109), .c(\a[8] ), .d(\b[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor042aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  norb02aa1n03x5               g018(.a(new_n112), .b(new_n113), .out0(new_n114));
  nor042aa1n06x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  norb02aa1n03x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nor022aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  tech160nm_fiaoi012aa1n04x5   g023(.a(new_n118), .b(\a[7] ), .c(\b[6] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n117), .b(new_n119), .o1(new_n120));
  nano32aa1n03x5               g025(.a(new_n120), .b(new_n114), .c(new_n111), .d(new_n108), .out0(new_n121));
  nand02aa1d04x5               g026(.a(new_n121), .b(new_n107), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n110), .b(new_n109), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  nor002aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  inv000aa1n06x5               g030(.a(new_n115), .o1(new_n126));
  tech160nm_fioaoi03aa1n03p5x5 g031(.a(\a[6] ), .b(\b[5] ), .c(new_n126), .o1(new_n127));
  nanp03aa1n02x5               g032(.a(new_n127), .b(new_n111), .c(new_n119), .o1(new_n128));
  nona23aa1n02x4               g033(.a(new_n122), .b(new_n128), .c(new_n125), .d(new_n124), .out0(new_n129));
  xnrc02aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n129), .c(new_n97), .out0(\s[10] ));
  nand02aa1n03x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(new_n129), .b(new_n97), .o1(new_n135));
  oaoi03aa1n02x5               g040(.a(\a[10] ), .b(\b[9] ), .c(new_n135), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n133), .o1(new_n137));
  inv000aa1d42x5               g042(.a(\a[10] ), .o1(new_n138));
  oaib12aa1n02x5               g043(.a(new_n135), .b(\b[9] ), .c(new_n138), .out0(new_n139));
  aoi022aa1n02x5               g044(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n140));
  nanp03aa1n02x5               g045(.a(new_n139), .b(new_n137), .c(new_n140), .o1(new_n141));
  oa0012aa1n02x5               g046(.a(new_n141), .b(new_n136), .c(new_n134), .o(\s[11] ));
  aob012aa1n02x5               g047(.a(new_n137), .b(new_n139), .c(new_n140), .out0(new_n143));
  nor022aa1n03x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand22aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  aoib12aa1n02x5               g051(.a(new_n133), .b(new_n145), .c(new_n144), .out0(new_n147));
  aoi022aa1n02x5               g052(.a(new_n143), .b(new_n146), .c(new_n141), .d(new_n147), .o1(\s[12] ));
  aoi013aa1n06x4               g053(.a(new_n124), .b(new_n127), .c(new_n111), .d(new_n119), .o1(new_n149));
  nona23aa1n09x5               g054(.a(new_n132), .b(new_n145), .c(new_n144), .d(new_n133), .out0(new_n150));
  nanb02aa1n06x5               g055(.a(new_n125), .b(new_n97), .out0(new_n151));
  nor043aa1d12x5               g056(.a(new_n150), .b(new_n151), .c(new_n130), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  norp02aa1n02x5               g058(.a(\b[9] ), .b(\a[10] ), .o1(new_n154));
  oaih12aa1n02x5               g059(.a(new_n140), .b(new_n154), .c(new_n125), .o1(new_n155));
  nona22aa1n02x4               g060(.a(new_n155), .b(new_n144), .c(new_n133), .out0(new_n156));
  and002aa1n06x5               g061(.a(new_n156), .b(new_n145), .o(new_n157));
  inv000aa1n02x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n153), .c(new_n122), .d(new_n149), .o1(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[12] ), .b(\a[13] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(new_n122), .b(new_n149), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n161), .b(new_n157), .c(new_n162), .d(new_n152), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n159), .c(new_n161), .o1(\s[13] ));
  inv000aa1d42x5               g069(.a(\a[13] ), .o1(new_n165));
  inv000aa1d42x5               g070(.a(\b[12] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(new_n166), .b(new_n165), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(new_n159), .b(new_n161), .o1(new_n168));
  xnrc02aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .out0(new_n169));
  xobna2aa1n03x5               g074(.a(new_n169), .b(new_n168), .c(new_n167), .out0(\s[14] ));
  nona22aa1n03x5               g075(.a(new_n159), .b(new_n160), .c(new_n169), .out0(new_n171));
  aoi112aa1n02x7               g076(.a(\b[12] ), .b(\a[13] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n172));
  oab012aa1n04x5               g077(.a(new_n172), .b(\a[14] ), .c(\b[13] ), .out0(new_n173));
  nor042aa1d18x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nand02aa1d04x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  norb02aa1d27x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n171), .c(new_n173), .out0(\s[15] ));
  aob012aa1n02x5               g082(.a(new_n176), .b(new_n171), .c(new_n173), .out0(new_n178));
  nor002aa1n03x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanp02aa1n04x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  aoib12aa1n02x5               g086(.a(new_n174), .b(new_n180), .c(new_n179), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n174), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n176), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n171), .d(new_n173), .o1(new_n185));
  aoi022aa1n02x7               g090(.a(new_n185), .b(new_n181), .c(new_n178), .d(new_n182), .o1(\s[16] ));
  nona23aa1n09x5               g091(.a(new_n180), .b(new_n175), .c(new_n174), .d(new_n179), .out0(new_n187));
  nona32aa1n09x5               g092(.a(new_n152), .b(new_n187), .c(new_n169), .d(new_n160), .out0(new_n188));
  nanb02aa1n02x5               g093(.a(new_n188), .b(new_n162), .out0(new_n189));
  nanb03aa1n06x5               g094(.a(new_n179), .b(new_n180), .c(new_n175), .out0(new_n190));
  aoi022aa1n02x5               g095(.a(new_n166), .b(new_n165), .c(\a[12] ), .d(\b[11] ), .o1(new_n191));
  oai022aa1n02x5               g096(.a(new_n165), .b(new_n166), .c(\b[13] ), .d(\a[14] ), .o1(new_n192));
  tech160nm_fiaoi012aa1n04x5   g097(.a(new_n174), .b(\a[14] ), .c(\b[13] ), .o1(new_n193));
  nano23aa1n06x5               g098(.a(new_n190), .b(new_n192), .c(new_n191), .d(new_n193), .out0(new_n194));
  tech160nm_fiaoi012aa1n03p5x5 g099(.a(new_n179), .b(new_n174), .c(new_n180), .o1(new_n195));
  tech160nm_fioai012aa1n04x5   g100(.a(new_n195), .b(new_n187), .c(new_n173), .o1(new_n196));
  aoi012aa1n12x5               g101(.a(new_n196), .b(new_n194), .c(new_n156), .o1(new_n197));
  aoai13aa1n12x5               g102(.a(new_n197), .b(new_n188), .c(new_n122), .d(new_n149), .o1(new_n198));
  xorc02aa1n02x5               g103(.a(\a[17] ), .b(\b[16] ), .out0(new_n199));
  norp02aa1n02x5               g104(.a(new_n187), .b(new_n173), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n195), .out0(new_n201));
  aoi112aa1n02x5               g106(.a(new_n200), .b(new_n201), .c(new_n194), .d(new_n156), .o1(new_n202));
  aoi022aa1n02x5               g107(.a(new_n202), .b(new_n189), .c(new_n198), .d(new_n199), .o1(\s[17] ));
  inv000aa1d42x5               g108(.a(\a[17] ), .o1(new_n204));
  nanb02aa1n12x5               g109(.a(\b[16] ), .b(new_n204), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n198), .b(new_n199), .o1(new_n206));
  xorc02aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n206), .c(new_n205), .out0(\s[18] ));
  inv000aa1n06x5               g113(.a(\a[18] ), .o1(new_n209));
  xroi22aa1d06x4               g114(.a(new_n204), .b(\b[16] ), .c(new_n209), .d(\b[17] ), .out0(new_n210));
  oaoi03aa1n12x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n205), .o1(new_n211));
  xorc02aa1n12x5               g116(.a(\a[19] ), .b(\b[18] ), .out0(new_n212));
  aoai13aa1n03x5               g117(.a(new_n212), .b(new_n211), .c(new_n198), .d(new_n210), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(new_n212), .b(new_n211), .c(new_n198), .d(new_n210), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n12x5               g121(.a(\a[20] ), .b(\b[19] ), .out0(new_n217));
  orn002aa1n12x5               g122(.a(\a[19] ), .b(\b[18] ), .o(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  nanp02aa1n02x5               g124(.a(new_n213), .b(new_n218), .o1(new_n220));
  aoi022aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n213), .d(new_n219), .o1(\s[20] ));
  nand23aa1n03x5               g126(.a(new_n210), .b(new_n212), .c(new_n217), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  nanp03aa1d12x5               g128(.a(new_n211), .b(new_n212), .c(new_n217), .o1(new_n224));
  oao003aa1n09x5               g129(.a(\a[20] ), .b(\b[19] ), .c(new_n218), .carry(new_n225));
  nanp02aa1n02x5               g130(.a(new_n224), .b(new_n225), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n226), .c(new_n198), .d(new_n223), .o1(new_n228));
  nano22aa1n02x4               g133(.a(new_n227), .b(new_n224), .c(new_n225), .out0(new_n229));
  aobi12aa1n02x5               g134(.a(new_n229), .b(new_n198), .c(new_n223), .out0(new_n230));
  norb02aa1n02x5               g135(.a(new_n228), .b(new_n230), .out0(\s[21] ));
  xorc02aa1n02x5               g136(.a(\a[22] ), .b(\b[21] ), .out0(new_n232));
  norp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norp02aa1n02x5               g138(.a(new_n232), .b(new_n233), .o1(new_n234));
  inv040aa1d30x5               g139(.a(\a[21] ), .o1(new_n235));
  oaib12aa1n02x5               g140(.a(new_n228), .b(\b[20] ), .c(new_n235), .out0(new_n236));
  aoi022aa1n02x5               g141(.a(new_n236), .b(new_n232), .c(new_n228), .d(new_n234), .o1(\s[22] ));
  inv040aa1d32x5               g142(.a(\a[22] ), .o1(new_n238));
  xroi22aa1d06x4               g143(.a(new_n235), .b(\b[20] ), .c(new_n238), .d(\b[21] ), .out0(new_n239));
  inv020aa1n04x5               g144(.a(new_n239), .o1(new_n240));
  nona22aa1n06x5               g145(.a(new_n198), .b(new_n222), .c(new_n240), .out0(new_n241));
  oai022aa1n02x5               g146(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n242));
  oaib12aa1n06x5               g147(.a(new_n242), .b(new_n238), .c(\b[21] ), .out0(new_n243));
  aoai13aa1n12x5               g148(.a(new_n243), .b(new_n240), .c(new_n224), .d(new_n225), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  xorc02aa1n12x5               g150(.a(\a[23] ), .b(\b[22] ), .out0(new_n246));
  aob012aa1n03x5               g151(.a(new_n246), .b(new_n241), .c(new_n245), .out0(new_n247));
  nanb02aa1n02x5               g152(.a(new_n246), .b(new_n243), .out0(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n226), .c(new_n239), .o1(new_n249));
  aobi12aa1n02x7               g154(.a(new_n247), .b(new_n249), .c(new_n241), .out0(\s[23] ));
  xorc02aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  nor042aa1n03x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  norp02aa1n02x5               g157(.a(new_n251), .b(new_n252), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n252), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n246), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n254), .b(new_n255), .c(new_n241), .d(new_n245), .o1(new_n256));
  aoi022aa1n02x7               g161(.a(new_n256), .b(new_n251), .c(new_n247), .d(new_n253), .o1(\s[24] ));
  and002aa1n06x5               g162(.a(new_n251), .b(new_n246), .o(new_n258));
  nano22aa1n02x5               g163(.a(new_n222), .b(new_n258), .c(new_n239), .out0(new_n259));
  nand22aa1n03x5               g164(.a(new_n198), .b(new_n259), .o1(new_n260));
  oaoi03aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n254), .o1(new_n261));
  aoi012aa1n02x5               g166(.a(new_n261), .b(new_n244), .c(new_n258), .o1(new_n262));
  nanp02aa1n03x5               g167(.a(new_n260), .b(new_n262), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  aoi112aa1n02x5               g169(.a(new_n264), .b(new_n261), .c(new_n244), .d(new_n258), .o1(new_n265));
  aoi022aa1n02x5               g170(.a(new_n263), .b(new_n264), .c(new_n260), .d(new_n265), .o1(\s[25] ));
  nanp02aa1n02x5               g171(.a(new_n263), .b(new_n264), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n268), .b(new_n269), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n269), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n264), .o1(new_n272));
  aoai13aa1n02x5               g177(.a(new_n271), .b(new_n272), .c(new_n260), .d(new_n262), .o1(new_n273));
  aoi022aa1n03x5               g178(.a(new_n273), .b(new_n268), .c(new_n267), .d(new_n270), .o1(\s[26] ));
  and002aa1n02x5               g179(.a(new_n268), .b(new_n264), .o(new_n275));
  aoai13aa1n09x5               g180(.a(new_n275), .b(new_n261), .c(new_n244), .d(new_n258), .o1(new_n276));
  nano32aa1n03x7               g181(.a(new_n222), .b(new_n275), .c(new_n239), .d(new_n258), .out0(new_n277));
  tech160nm_finand02aa1n05x5   g182(.a(new_n198), .b(new_n277), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[25] ), .b(\a[26] ), .o1(new_n279));
  oai022aa1n02x5               g184(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(new_n280), .b(new_n279), .o1(new_n281));
  nanp03aa1n09x5               g186(.a(new_n278), .b(new_n276), .c(new_n281), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  aoi122aa1n02x5               g188(.a(new_n283), .b(new_n279), .c(new_n280), .d(new_n198), .e(new_n277), .o1(new_n284));
  aoi022aa1n02x5               g189(.a(new_n284), .b(new_n276), .c(new_n282), .d(new_n283), .o1(\s[27] ));
  nanp02aa1n03x5               g190(.a(new_n282), .b(new_n283), .o1(new_n286));
  tech160nm_fixorc02aa1n04x5   g191(.a(\a[28] ), .b(\b[27] ), .out0(new_n287));
  norp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n287), .b(new_n288), .o1(new_n289));
  aoi022aa1n06x5               g194(.a(new_n198), .b(new_n277), .c(new_n279), .d(new_n280), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n288), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n283), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n291), .b(new_n292), .c(new_n290), .d(new_n276), .o1(new_n293));
  aoi022aa1n03x5               g198(.a(new_n293), .b(new_n287), .c(new_n286), .d(new_n289), .o1(\s[28] ));
  and002aa1n02x5               g199(.a(new_n287), .b(new_n283), .o(new_n295));
  nand22aa1n03x5               g200(.a(new_n282), .b(new_n295), .o1(new_n296));
  xorc02aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .out0(new_n297));
  inv000aa1d42x5               g202(.a(\a[28] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(\b[27] ), .o1(new_n299));
  aoi112aa1n02x5               g204(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n300));
  aoi112aa1n02x5               g205(.a(new_n297), .b(new_n300), .c(new_n298), .d(new_n299), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n295), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n302), .c(new_n290), .d(new_n276), .o1(new_n304));
  aoi022aa1n03x5               g209(.a(new_n304), .b(new_n297), .c(new_n296), .d(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g210(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g211(.a(new_n292), .b(new_n287), .c(new_n297), .out0(new_n307));
  nanp02aa1n03x5               g212(.a(new_n282), .b(new_n307), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .out0(new_n309));
  aoi012aa1n02x5               g214(.a(new_n303), .b(\a[29] ), .c(\b[28] ), .o1(new_n310));
  oabi12aa1n02x5               g215(.a(new_n309), .b(\a[29] ), .c(\b[28] ), .out0(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n310), .o1(new_n312));
  inv000aa1n02x5               g217(.a(new_n307), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n303), .carry(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n313), .c(new_n290), .d(new_n276), .o1(new_n315));
  aoi022aa1n03x5               g220(.a(new_n315), .b(new_n309), .c(new_n308), .d(new_n312), .o1(\s[30] ));
  nano32aa1n02x4               g221(.a(new_n292), .b(new_n309), .c(new_n287), .d(new_n297), .out0(new_n317));
  nanp02aa1n03x5               g222(.a(new_n282), .b(new_n317), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[31] ), .b(\b[30] ), .out0(new_n319));
  nanp02aa1n02x5               g224(.a(\b[29] ), .b(\a[30] ), .o1(new_n320));
  oai022aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(\b[29] ), .d(\a[30] ), .o1(new_n321));
  oaoi13aa1n02x5               g226(.a(new_n319), .b(new_n320), .c(new_n310), .d(new_n321), .o1(new_n322));
  inv000aa1n02x5               g227(.a(new_n317), .o1(new_n323));
  oai012aa1n02x5               g228(.a(new_n320), .b(new_n310), .c(new_n321), .o1(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n323), .c(new_n290), .d(new_n276), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n325), .b(new_n319), .c(new_n318), .d(new_n322), .o1(\s[31] ));
  norp02aa1n02x5               g231(.a(new_n105), .b(new_n99), .o1(new_n327));
  aboi22aa1n03x5               g232(.a(new_n99), .b(new_n100), .c(new_n103), .d(new_n104), .out0(new_n328));
  norp02aa1n02x5               g233(.a(new_n328), .b(new_n327), .o1(\s[3] ));
  inv000aa1n03x5               g234(.a(new_n327), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[4] ), .b(\b[3] ), .out0(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n331), .b(new_n330), .c(new_n103), .out0(\s[4] ));
  xobna2aa1n03x5               g237(.a(new_n117), .b(new_n107), .c(new_n108), .out0(\s[5] ));
  nanp03aa1n02x5               g238(.a(new_n107), .b(new_n108), .c(new_n117), .o1(new_n334));
  xnbna2aa1n03x5               g239(.a(new_n114), .b(new_n334), .c(new_n126), .out0(\s[6] ));
  and003aa1n02x5               g240(.a(new_n114), .b(new_n117), .c(new_n108), .o(new_n336));
  xorc02aa1n02x5               g241(.a(\a[7] ), .b(\b[6] ), .out0(new_n337));
  aoai13aa1n02x5               g242(.a(new_n337), .b(new_n127), .c(new_n336), .d(new_n107), .o1(new_n338));
  aoi112aa1n02x5               g243(.a(new_n337), .b(new_n127), .c(new_n336), .d(new_n107), .o1(new_n339));
  norb02aa1n02x5               g244(.a(new_n338), .b(new_n339), .out0(\s[7] ));
  xorc02aa1n02x5               g245(.a(\a[8] ), .b(\b[7] ), .out0(new_n341));
  xnbna2aa1n03x5               g246(.a(new_n341), .b(new_n338), .c(new_n123), .out0(\s[8] ));
  nona23aa1n02x4               g247(.a(new_n122), .b(new_n128), .c(new_n151), .d(new_n124), .out0(new_n343));
  aob012aa1n02x5               g248(.a(new_n343), .b(new_n162), .c(new_n151), .out0(\s[9] ));
endmodule


