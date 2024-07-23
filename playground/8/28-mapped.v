// Benchmark "adder" written by ABC on Wed Jul 17 16:16:53 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n326, new_n327, new_n330, new_n332, new_n333, new_n334,
    new_n335;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n03x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  aoi012aa1n12x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  inv040aa1n02x5               g005(.a(new_n100), .o1(new_n101));
  nor022aa1n12x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor042aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano23aa1n06x5               g010(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n106));
  inv000aa1n02x5               g011(.a(new_n104), .o1(new_n107));
  oaoi03aa1n02x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .o1(new_n108));
  tech160nm_fiaoi012aa1n05x5   g013(.a(new_n108), .b(new_n106), .c(new_n101), .o1(new_n109));
  nor042aa1n06x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n08x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor002aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand42aa1n03x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nano23aa1n03x5               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  norp02aa1n04x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nand22aa1n04x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norb02aa1n06x4               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  xorc02aa1n12x5               g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  nand23aa1n03x5               g023(.a(new_n114), .b(new_n117), .c(new_n118), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[8] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[7] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n110), .o1(new_n122));
  aoai13aa1n03x5               g027(.a(new_n111), .b(new_n115), .c(new_n112), .d(new_n116), .o1(new_n123));
  nanp02aa1n06x5               g028(.a(new_n123), .b(new_n122), .o1(new_n124));
  oaoi03aa1n09x5               g029(.a(new_n120), .b(new_n121), .c(new_n124), .o1(new_n125));
  oai012aa1d24x5               g030(.a(new_n125), .b(new_n109), .c(new_n119), .o1(new_n126));
  nor042aa1n03x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  xorc02aa1n12x5               g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  aoi112aa1n02x5               g034(.a(new_n127), .b(new_n129), .c(new_n126), .d(new_n128), .o1(new_n130));
  aoai13aa1n02x5               g035(.a(new_n129), .b(new_n127), .c(new_n126), .d(new_n128), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(\s[10] ));
  inv000aa1d42x5               g037(.a(\a[10] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(\b[8] ), .o1(new_n134));
  xroi22aa1d04x5               g039(.a(new_n133), .b(\b[9] ), .c(new_n134), .d(\a[9] ), .out0(new_n135));
  inv000aa1d42x5               g040(.a(\b[9] ), .o1(new_n136));
  oao003aa1n02x5               g041(.a(new_n133), .b(new_n136), .c(new_n127), .carry(new_n137));
  xorc02aa1n12x5               g042(.a(\a[11] ), .b(\b[10] ), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n126), .d(new_n135), .o1(new_n139));
  aoi112aa1n02x5               g044(.a(new_n138), .b(new_n137), .c(new_n126), .d(new_n135), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(\s[11] ));
  nor042aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  tech160nm_finand02aa1n05x5   g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanb02aa1n12x5               g048(.a(new_n142), .b(new_n143), .out0(new_n144));
  inv040aa1n02x5               g049(.a(new_n144), .o1(new_n145));
  nor002aa1n02x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  aoib12aa1n02x5               g051(.a(new_n146), .b(new_n143), .c(new_n142), .out0(new_n147));
  oai012aa1n02x5               g052(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .o1(new_n148));
  aoi022aa1n02x5               g053(.a(new_n148), .b(new_n145), .c(new_n139), .d(new_n147), .o1(\s[12] ));
  nona23aa1n03x5               g054(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n150));
  oabi12aa1n06x5               g055(.a(new_n108), .b(new_n150), .c(new_n100), .out0(new_n151));
  nanb02aa1n03x5               g056(.a(new_n119), .b(new_n151), .out0(new_n152));
  nano32aa1d12x5               g057(.a(new_n144), .b(new_n138), .c(new_n129), .d(new_n128), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  tech160nm_fioaoi03aa1n04x5   g059(.a(new_n133), .b(new_n136), .c(new_n127), .o1(new_n155));
  tech160nm_fixnrc02aa1n02p5x5 g060(.a(\b[10] ), .b(\a[11] ), .out0(new_n156));
  aoi012aa1n06x5               g061(.a(new_n142), .b(new_n146), .c(new_n143), .o1(new_n157));
  oai013aa1n09x5               g062(.a(new_n157), .b(new_n155), .c(new_n156), .d(new_n144), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n154), .c(new_n152), .d(new_n125), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n24x5               g066(.a(\a[13] ), .b(\b[12] ), .o(new_n162));
  xorc02aa1n12x5               g067(.a(\a[13] ), .b(\b[12] ), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n158), .c(new_n126), .d(new_n153), .o1(new_n164));
  xorc02aa1n12x5               g069(.a(\a[14] ), .b(\b[13] ), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n162), .out0(\s[14] ));
  nanp02aa1n12x5               g071(.a(new_n165), .b(new_n163), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n158), .c(new_n126), .d(new_n153), .o1(new_n169));
  oaoi03aa1n12x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n162), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  tech160nm_fixorc02aa1n04x5   g076(.a(\a[15] ), .b(\b[14] ), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n171), .out0(\s[15] ));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n170), .c(new_n160), .d(new_n168), .o1(new_n174));
  xorc02aa1n12x5               g079(.a(\a[16] ), .b(\b[15] ), .out0(new_n175));
  nor042aa1n06x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norp02aa1n02x5               g081(.a(new_n175), .b(new_n176), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n176), .o1(new_n178));
  xnrc02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n178), .b(new_n179), .c(new_n169), .d(new_n171), .o1(new_n180));
  aoi022aa1n02x5               g085(.a(new_n180), .b(new_n175), .c(new_n174), .d(new_n177), .o1(\s[16] ));
  nano22aa1n03x5               g086(.a(new_n167), .b(new_n172), .c(new_n175), .out0(new_n182));
  nand02aa1n02x5               g087(.a(new_n153), .b(new_n182), .o1(new_n183));
  nand03aa1n02x5               g088(.a(new_n170), .b(new_n172), .c(new_n175), .o1(new_n184));
  oaoi03aa1n03x5               g089(.a(\a[16] ), .b(\b[15] ), .c(new_n178), .o1(new_n185));
  nanb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  aoi012aa1n06x5               g091(.a(new_n186), .b(new_n158), .c(new_n182), .o1(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n183), .c(new_n152), .d(new_n125), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(\b[16] ), .b(new_n190), .out0(new_n191));
  xnrc02aa1n02x5               g096(.a(\b[12] ), .b(\a[13] ), .out0(new_n192));
  nona23aa1n03x5               g097(.a(new_n175), .b(new_n165), .c(new_n192), .d(new_n179), .out0(new_n193));
  nano32aa1n03x7               g098(.a(new_n193), .b(new_n135), .c(new_n138), .d(new_n145), .out0(new_n194));
  nanp03aa1n02x5               g099(.a(new_n137), .b(new_n145), .c(new_n138), .o1(new_n195));
  aoi013aa1n06x5               g100(.a(new_n185), .b(new_n170), .c(new_n172), .d(new_n175), .o1(new_n196));
  aoai13aa1n09x5               g101(.a(new_n196), .b(new_n193), .c(new_n195), .d(new_n157), .o1(new_n197));
  xorc02aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  aoai13aa1n03x5               g103(.a(new_n198), .b(new_n197), .c(new_n126), .d(new_n194), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[18] ), .b(\b[17] ), .out0(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n199), .c(new_n191), .out0(\s[18] ));
  inv000aa1d42x5               g106(.a(\a[18] ), .o1(new_n202));
  xroi22aa1d04x5               g107(.a(new_n190), .b(\b[16] ), .c(new_n202), .d(\b[17] ), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n197), .c(new_n126), .d(new_n194), .o1(new_n204));
  norp02aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  aoi112aa1n09x5               g110(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n206));
  nor042aa1n02x5               g111(.a(new_n206), .b(new_n205), .o1(new_n207));
  xnrc02aa1n12x5               g112(.a(\b[18] ), .b(\a[19] ), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n204), .c(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n02x5               g116(.a(new_n207), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n209), .b(new_n212), .c(new_n188), .d(new_n203), .o1(new_n213));
  xnrc02aa1n12x5               g118(.a(\b[19] ), .b(\a[20] ), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nor002aa1d32x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n214), .b(new_n216), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n216), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n208), .c(new_n204), .d(new_n207), .o1(new_n219));
  aoi022aa1n03x5               g124(.a(new_n219), .b(new_n215), .c(new_n213), .d(new_n217), .o1(\s[20] ));
  nano23aa1n02x4               g125(.a(new_n214), .b(new_n208), .c(new_n200), .d(new_n198), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n197), .c(new_n126), .d(new_n194), .o1(new_n222));
  oao003aa1n02x5               g127(.a(\a[20] ), .b(\b[19] ), .c(new_n218), .carry(new_n223));
  oai013aa1d12x5               g128(.a(new_n223), .b(new_n207), .c(new_n208), .d(new_n214), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[20] ), .b(\a[21] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n225), .out0(\s[21] ));
  aoai13aa1n03x5               g133(.a(new_n227), .b(new_n224), .c(new_n188), .d(new_n221), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[22] ), .b(\b[21] ), .out0(new_n230));
  nor042aa1n03x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  norp02aa1n02x5               g136(.a(new_n230), .b(new_n231), .o1(new_n232));
  inv040aa1n03x5               g137(.a(new_n231), .o1(new_n233));
  aoai13aa1n02x5               g138(.a(new_n233), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n234));
  aoi022aa1n03x5               g139(.a(new_n234), .b(new_n230), .c(new_n229), .d(new_n232), .o1(\s[22] ));
  nor042aa1n02x5               g140(.a(new_n214), .b(new_n208), .o1(new_n236));
  xnrc02aa1n02x5               g141(.a(\b[21] ), .b(\a[22] ), .out0(new_n237));
  norp02aa1n02x5               g142(.a(new_n237), .b(new_n226), .o1(new_n238));
  and003aa1n02x5               g143(.a(new_n203), .b(new_n238), .c(new_n236), .o(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n197), .c(new_n126), .d(new_n194), .o1(new_n240));
  tech160nm_fioaoi03aa1n03p5x5 g145(.a(\a[22] ), .b(\b[21] ), .c(new_n233), .o1(new_n241));
  tech160nm_fiaoi012aa1n04x5   g146(.a(new_n241), .b(new_n224), .c(new_n238), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(new_n240), .b(new_n242), .o1(new_n243));
  xorc02aa1n12x5               g148(.a(\a[23] ), .b(\b[22] ), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n244), .b(new_n241), .c(new_n224), .d(new_n238), .o1(new_n245));
  aoi022aa1n02x5               g150(.a(new_n243), .b(new_n244), .c(new_n240), .d(new_n245), .o1(\s[23] ));
  inv000aa1n02x5               g151(.a(new_n242), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n244), .b(new_n247), .c(new_n188), .d(new_n239), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  nor042aa1n06x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n250), .o1(new_n252));
  xnrc02aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .out0(new_n253));
  aoai13aa1n02x7               g158(.a(new_n252), .b(new_n253), .c(new_n240), .d(new_n242), .o1(new_n254));
  aoi022aa1n03x5               g159(.a(new_n254), .b(new_n249), .c(new_n248), .d(new_n251), .o1(\s[24] ));
  nona23aa1n08x5               g160(.a(new_n244), .b(new_n249), .c(new_n237), .d(new_n226), .out0(new_n256));
  nano22aa1n02x4               g161(.a(new_n256), .b(new_n203), .c(new_n236), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n197), .c(new_n126), .d(new_n194), .o1(new_n258));
  nano32aa1n02x4               g163(.a(new_n226), .b(new_n249), .c(new_n230), .d(new_n244), .out0(new_n259));
  oaoi03aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .c(new_n252), .o1(new_n260));
  aoi013aa1n03x5               g165(.a(new_n260), .b(new_n241), .c(new_n244), .d(new_n249), .o1(new_n261));
  inv030aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  tech160nm_fiaoi012aa1n02p5x5 g167(.a(new_n262), .b(new_n224), .c(new_n259), .o1(new_n263));
  nanp02aa1n02x5               g168(.a(new_n258), .b(new_n263), .o1(new_n264));
  xorc02aa1n12x5               g169(.a(\a[25] ), .b(\b[24] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n262), .b(new_n265), .c(new_n259), .d(new_n224), .o1(new_n266));
  aoi022aa1n02x5               g171(.a(new_n264), .b(new_n265), .c(new_n258), .d(new_n266), .o1(\s[25] ));
  nona22aa1n03x5               g172(.a(new_n212), .b(new_n208), .c(new_n214), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n261), .b(new_n256), .c(new_n268), .d(new_n223), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n265), .b(new_n269), .c(new_n188), .d(new_n257), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  nor042aa1n06x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n271), .b(new_n272), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n272), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n265), .o1(new_n275));
  aoai13aa1n02x7               g180(.a(new_n274), .b(new_n275), .c(new_n258), .d(new_n263), .o1(new_n276));
  aoi022aa1n02x7               g181(.a(new_n276), .b(new_n271), .c(new_n270), .d(new_n273), .o1(\s[26] ));
  and002aa1n02x5               g182(.a(new_n271), .b(new_n265), .o(new_n278));
  nano32aa1n03x7               g183(.a(new_n256), .b(new_n278), .c(new_n203), .d(new_n236), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n197), .c(new_n126), .d(new_n194), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .c(new_n274), .carry(new_n281));
  inv000aa1n02x5               g186(.a(new_n281), .o1(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n282), .b(new_n269), .c(new_n278), .o1(new_n283));
  xorc02aa1n12x5               g188(.a(\a[27] ), .b(\b[26] ), .out0(new_n284));
  xnbna2aa1n03x5               g189(.a(new_n284), .b(new_n280), .c(new_n283), .out0(\s[27] ));
  aoai13aa1n03x5               g190(.a(new_n278), .b(new_n262), .c(new_n224), .d(new_n259), .o1(new_n286));
  nand42aa1n02x5               g191(.a(new_n286), .b(new_n281), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n284), .b(new_n287), .c(new_n188), .d(new_n279), .o1(new_n288));
  xorc02aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .out0(new_n289));
  norp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n289), .b(new_n290), .o1(new_n291));
  inv000aa1n06x5               g196(.a(new_n290), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n284), .o1(new_n293));
  aoai13aa1n02x7               g198(.a(new_n292), .b(new_n293), .c(new_n280), .d(new_n283), .o1(new_n294));
  aoi022aa1n03x5               g199(.a(new_n294), .b(new_n289), .c(new_n288), .d(new_n291), .o1(\s[28] ));
  and002aa1n02x5               g200(.a(new_n289), .b(new_n284), .o(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n287), .c(new_n188), .d(new_n279), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n296), .o1(new_n298));
  oaoi03aa1n12x5               g203(.a(\a[28] ), .b(\b[27] ), .c(new_n292), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n299), .o1(new_n300));
  aoai13aa1n02x7               g205(.a(new_n300), .b(new_n298), .c(new_n280), .d(new_n283), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n299), .b(new_n302), .o1(new_n303));
  aoi022aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n297), .d(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g210(.a(new_n293), .b(new_n289), .c(new_n302), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n287), .c(new_n188), .d(new_n279), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n306), .o1(new_n308));
  inv000aa1d42x5               g213(.a(\a[29] ), .o1(new_n309));
  inv000aa1d42x5               g214(.a(\b[28] ), .o1(new_n310));
  oaoi03aa1n02x5               g215(.a(new_n309), .b(new_n310), .c(new_n299), .o1(new_n311));
  aoai13aa1n02x7               g216(.a(new_n311), .b(new_n308), .c(new_n280), .d(new_n283), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .out0(new_n313));
  oabi12aa1n02x5               g218(.a(new_n313), .b(\a[29] ), .c(\b[28] ), .out0(new_n314));
  oaoi13aa1n02x5               g219(.a(new_n314), .b(new_n299), .c(new_n309), .d(new_n310), .o1(new_n315));
  aoi022aa1n03x5               g220(.a(new_n312), .b(new_n313), .c(new_n307), .d(new_n315), .o1(\s[30] ));
  nano32aa1d12x5               g221(.a(new_n293), .b(new_n313), .c(new_n289), .d(new_n302), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n287), .c(new_n188), .d(new_n279), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[31] ), .b(\b[30] ), .out0(new_n319));
  oao003aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .c(new_n311), .carry(new_n320));
  norb02aa1n02x5               g225(.a(new_n320), .b(new_n319), .out0(new_n321));
  inv000aa1d42x5               g226(.a(new_n317), .o1(new_n322));
  aoai13aa1n03x5               g227(.a(new_n320), .b(new_n322), .c(new_n280), .d(new_n283), .o1(new_n323));
  aoi022aa1n03x5               g228(.a(new_n323), .b(new_n319), .c(new_n318), .d(new_n321), .o1(\s[31] ));
  xnbna2aa1n03x5               g229(.a(new_n100), .b(new_n105), .c(new_n107), .out0(\s[3] ));
  inv000aa1d42x5               g230(.a(new_n102), .o1(new_n326));
  aoi122aa1n02x5               g231(.a(new_n104), .b(new_n326), .c(new_n103), .d(new_n101), .e(new_n105), .o1(new_n327));
  aoi012aa1n02x5               g232(.a(new_n327), .b(new_n326), .c(new_n151), .o1(\s[4] ));
  xorb03aa1n02x5               g233(.a(new_n151), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g234(.a(new_n112), .b(new_n151), .c(new_n113), .o1(new_n330));
  xnrc02aa1n02x5               g235(.a(new_n330), .b(new_n117), .out0(\s[6] ));
  nanp02aa1n02x5               g236(.a(new_n330), .b(new_n117), .o1(new_n332));
  nano22aa1n02x4               g237(.a(new_n110), .b(new_n111), .c(new_n116), .out0(new_n333));
  nanp02aa1n02x5               g238(.a(new_n332), .b(new_n333), .o1(new_n334));
  aoi022aa1n02x5               g239(.a(new_n332), .b(new_n116), .c(new_n122), .d(new_n111), .o1(new_n335));
  norb02aa1n02x5               g240(.a(new_n334), .b(new_n335), .out0(\s[7] ));
  xnbna2aa1n03x5               g241(.a(new_n118), .b(new_n334), .c(new_n122), .out0(\s[8] ));
  xorb03aa1n02x5               g242(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


