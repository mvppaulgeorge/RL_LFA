// Benchmark "adder" written by ABC on Wed Jul 17 23:54:01 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n176, new_n177, new_n178, new_n179, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n208, new_n209, new_n210, new_n211, new_n212, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n242, new_n243, new_n244, new_n245, new_n246,
    new_n247, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n268, new_n269, new_n270,
    new_n271, new_n272, new_n273, new_n274, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n294, new_n297, new_n298,
    new_n300, new_n301, new_n302, new_n304;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  and002aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o(new_n98));
  nand42aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oaih12aa1n06x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n09x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n09x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n24x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fiaoi012aa1n03p5x5 g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oaih12aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor022aa1n08x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  tech160nm_fixnrc02aa1n04x5   g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  tech160nm_fixnrc02aa1n05x5   g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n03x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  and002aa1n12x5               g023(.a(\b[5] ), .b(\a[6] ), .o(new_n119));
  oai022aa1d18x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  nanb02aa1n06x5               g025(.a(new_n119), .b(new_n120), .out0(new_n121));
  inv040aa1n03x5               g026(.a(new_n112), .o1(new_n122));
  oaoi03aa1n03x5               g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .o1(new_n123));
  oabi12aa1n06x5               g028(.a(new_n123), .b(new_n114), .c(new_n121), .out0(new_n124));
  inv000aa1n02x5               g029(.a(new_n124), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n97), .b(new_n98), .c(new_n118), .d(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  tech160nm_fixorc02aa1n05x5   g032(.a(\a[10] ), .b(\b[9] ), .out0(new_n128));
  nor042aa1n06x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand42aa1d28x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  oaoi03aa1n12x5               g036(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n132));
  aoai13aa1n06x5               g037(.a(new_n131), .b(new_n132), .c(new_n126), .d(new_n128), .o1(new_n133));
  aoi112aa1n02x5               g038(.a(new_n131), .b(new_n132), .c(new_n126), .d(new_n128), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(\s[11] ));
  nor042aa1n06x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand42aa1n16x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  nona22aa1n02x4               g043(.a(new_n133), .b(new_n138), .c(new_n129), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n138), .o1(new_n140));
  oaoi13aa1n06x5               g045(.a(new_n140), .b(new_n133), .c(\a[11] ), .d(\b[10] ), .o1(new_n141));
  norb02aa1n03x4               g046(.a(new_n139), .b(new_n141), .out0(\s[12] ));
  nano23aa1n09x5               g047(.a(new_n129), .b(new_n136), .c(new_n137), .d(new_n130), .out0(new_n143));
  tech160nm_fixorc02aa1n03p5x5 g048(.a(\a[9] ), .b(\b[8] ), .out0(new_n144));
  and003aa1n02x5               g049(.a(new_n143), .b(new_n128), .c(new_n144), .o(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n146));
  ao0012aa1n03x7               g051(.a(new_n136), .b(new_n129), .c(new_n137), .o(new_n147));
  aoi012aa1n06x5               g052(.a(new_n147), .b(new_n143), .c(new_n132), .o1(new_n148));
  xorc02aa1n12x5               g053(.a(\a[13] ), .b(\b[12] ), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n146), .c(new_n148), .out0(\s[13] ));
  orn002aa1n03x5               g055(.a(\a[13] ), .b(\b[12] ), .o(new_n151));
  aob012aa1n02x5               g056(.a(new_n149), .b(new_n146), .c(new_n148), .out0(new_n152));
  xorc02aa1n12x5               g057(.a(\a[14] ), .b(\b[13] ), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n152), .c(new_n151), .out0(\s[14] ));
  nanp02aa1n02x5               g059(.a(new_n153), .b(new_n149), .o1(new_n155));
  oao003aa1n03x5               g060(.a(\a[14] ), .b(\b[13] ), .c(new_n151), .carry(new_n156));
  aoai13aa1n04x5               g061(.a(new_n156), .b(new_n155), .c(new_n146), .d(new_n148), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n09x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nand02aa1d24x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nor002aa1n04x5               g065(.a(\b[15] ), .b(\a[16] ), .o1(new_n161));
  nand02aa1d12x5               g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  aoi112aa1n02x5               g068(.a(new_n163), .b(new_n159), .c(new_n157), .d(new_n160), .o1(new_n164));
  aoai13aa1n04x5               g069(.a(new_n163), .b(new_n159), .c(new_n157), .d(new_n160), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(\s[16] ));
  nano23aa1n06x5               g071(.a(new_n159), .b(new_n161), .c(new_n162), .d(new_n160), .out0(new_n167));
  nand03aa1n02x5               g072(.a(new_n167), .b(new_n149), .c(new_n153), .o1(new_n168));
  nano32aa1n03x7               g073(.a(new_n168), .b(new_n144), .c(new_n143), .d(new_n128), .out0(new_n169));
  aoai13aa1n12x5               g074(.a(new_n169), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n171));
  oaib12aa1n09x5               g076(.a(new_n171), .b(new_n156), .c(new_n167), .out0(new_n172));
  oab012aa1n09x5               g077(.a(new_n172), .b(new_n148), .c(new_n168), .out0(new_n173));
  nand02aa1d08x5               g078(.a(new_n170), .b(new_n173), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g080(.a(\a[18] ), .o1(new_n176));
  inv040aa1d32x5               g081(.a(\a[17] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\b[16] ), .o1(new_n178));
  oaoi03aa1n03x5               g083(.a(new_n177), .b(new_n178), .c(new_n174), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[17] ), .c(new_n176), .out0(\s[18] ));
  xroi22aa1d06x4               g085(.a(new_n177), .b(\b[16] ), .c(new_n176), .d(\b[17] ), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n178), .b(new_n177), .o1(new_n182));
  oaoi03aa1n12x5               g087(.a(\a[18] ), .b(\b[17] ), .c(new_n182), .o1(new_n183));
  nor002aa1n16x5               g088(.a(\b[18] ), .b(\a[19] ), .o1(new_n184));
  nand02aa1n06x5               g089(.a(\b[18] ), .b(\a[19] ), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n183), .c(new_n174), .d(new_n181), .o1(new_n187));
  aoi112aa1n02x5               g092(.a(new_n186), .b(new_n183), .c(new_n174), .d(new_n181), .o1(new_n188));
  norb02aa1n02x7               g093(.a(new_n187), .b(new_n188), .out0(\s[19] ));
  xnrc02aa1n02x5               g094(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n16x5               g095(.a(\b[19] ), .b(\a[20] ), .o1(new_n191));
  nand42aa1n16x5               g096(.a(\b[19] ), .b(\a[20] ), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  nona22aa1n02x5               g098(.a(new_n187), .b(new_n193), .c(new_n184), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n193), .o1(new_n195));
  oaoi13aa1n06x5               g100(.a(new_n195), .b(new_n187), .c(\a[19] ), .d(\b[18] ), .o1(new_n196));
  norb02aa1n03x4               g101(.a(new_n194), .b(new_n196), .out0(\s[20] ));
  nano23aa1n09x5               g102(.a(new_n184), .b(new_n191), .c(new_n192), .d(new_n185), .out0(new_n198));
  nanp02aa1n02x5               g103(.a(new_n181), .b(new_n198), .o1(new_n199));
  oai022aa1n04x5               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  oaib12aa1n09x5               g105(.a(new_n200), .b(new_n176), .c(\b[17] ), .out0(new_n201));
  nona23aa1n12x5               g106(.a(new_n192), .b(new_n185), .c(new_n184), .d(new_n191), .out0(new_n202));
  aoi012aa1n12x5               g107(.a(new_n191), .b(new_n184), .c(new_n192), .o1(new_n203));
  oai012aa1n18x5               g108(.a(new_n203), .b(new_n202), .c(new_n201), .o1(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  aoai13aa1n04x5               g110(.a(new_n205), .b(new_n199), .c(new_n170), .d(new_n173), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g112(.a(\b[20] ), .b(\a[21] ), .o1(new_n208));
  xorc02aa1n02x5               g113(.a(\a[21] ), .b(\b[20] ), .out0(new_n209));
  xorc02aa1n02x5               g114(.a(\a[22] ), .b(\b[21] ), .out0(new_n210));
  aoi112aa1n02x5               g115(.a(new_n208), .b(new_n210), .c(new_n206), .d(new_n209), .o1(new_n211));
  aoai13aa1n03x5               g116(.a(new_n210), .b(new_n208), .c(new_n206), .d(new_n209), .o1(new_n212));
  norb02aa1n02x7               g117(.a(new_n212), .b(new_n211), .out0(\s[22] ));
  inv000aa1d42x5               g118(.a(\a[21] ), .o1(new_n214));
  inv040aa1d32x5               g119(.a(\a[22] ), .o1(new_n215));
  xroi22aa1d06x4               g120(.a(new_n214), .b(\b[20] ), .c(new_n215), .d(\b[21] ), .out0(new_n216));
  nanp03aa1n03x5               g121(.a(new_n216), .b(new_n181), .c(new_n198), .o1(new_n217));
  inv000aa1d42x5               g122(.a(\b[21] ), .o1(new_n218));
  oaoi03aa1n09x5               g123(.a(new_n215), .b(new_n218), .c(new_n208), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoi012aa1n02x5               g125(.a(new_n220), .b(new_n204), .c(new_n216), .o1(new_n221));
  aoai13aa1n04x5               g126(.a(new_n221), .b(new_n217), .c(new_n170), .d(new_n173), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g128(.a(\b[22] ), .b(\a[23] ), .o1(new_n224));
  tech160nm_fixorc02aa1n05x5   g129(.a(\a[23] ), .b(\b[22] ), .out0(new_n225));
  xorc02aa1n03x5               g130(.a(\a[24] ), .b(\b[23] ), .out0(new_n226));
  aoi112aa1n02x5               g131(.a(new_n224), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n228));
  norb02aa1n02x7               g133(.a(new_n228), .b(new_n227), .out0(\s[24] ));
  and002aa1n02x5               g134(.a(new_n226), .b(new_n225), .o(new_n230));
  inv000aa1n02x5               g135(.a(new_n230), .o1(new_n231));
  nano32aa1n02x4               g136(.a(new_n231), .b(new_n216), .c(new_n181), .d(new_n198), .out0(new_n232));
  inv020aa1n03x5               g137(.a(new_n203), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n216), .b(new_n233), .c(new_n198), .d(new_n183), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n235));
  oab012aa1n02x4               g140(.a(new_n235), .b(\a[24] ), .c(\b[23] ), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n231), .c(new_n234), .d(new_n219), .o1(new_n237));
  aoi012aa1n03x5               g142(.a(new_n237), .b(new_n174), .c(new_n232), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[24] ), .b(\a[25] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  xnrc02aa1n02x5               g145(.a(new_n238), .b(new_n240), .out0(\s[25] ));
  nor042aa1n03x5               g146(.a(\b[24] ), .b(\a[25] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n240), .b(new_n237), .c(new_n174), .d(new_n232), .o1(new_n244));
  xnrc02aa1n12x5               g149(.a(\b[25] ), .b(\a[26] ), .out0(new_n245));
  nand43aa1n02x5               g150(.a(new_n244), .b(new_n243), .c(new_n245), .o1(new_n246));
  tech160nm_fiaoi012aa1n02p5x5 g151(.a(new_n245), .b(new_n244), .c(new_n243), .o1(new_n247));
  norb02aa1n03x4               g152(.a(new_n246), .b(new_n247), .out0(\s[26] ));
  nor042aa1n06x5               g153(.a(new_n245), .b(new_n239), .o1(new_n249));
  nano22aa1n03x7               g154(.a(new_n217), .b(new_n230), .c(new_n249), .out0(new_n250));
  nand22aa1n09x5               g155(.a(new_n174), .b(new_n250), .o1(new_n251));
  oao003aa1n02x5               g156(.a(\a[26] ), .b(\b[25] ), .c(new_n243), .carry(new_n252));
  aobi12aa1n09x5               g157(.a(new_n252), .b(new_n237), .c(new_n249), .out0(new_n253));
  xorc02aa1n12x5               g158(.a(\a[27] ), .b(\b[26] ), .out0(new_n254));
  xnbna2aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n251), .out0(\s[27] ));
  norp02aa1n02x5               g160(.a(\b[26] ), .b(\a[27] ), .o1(new_n256));
  inv040aa1n03x5               g161(.a(new_n256), .o1(new_n257));
  aobi12aa1n06x5               g162(.a(new_n254), .b(new_n253), .c(new_n251), .out0(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[27] ), .b(\a[28] ), .out0(new_n259));
  nano22aa1n03x5               g164(.a(new_n258), .b(new_n257), .c(new_n259), .out0(new_n260));
  aobi12aa1n06x5               g165(.a(new_n250), .b(new_n170), .c(new_n173), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n230), .b(new_n220), .c(new_n204), .d(new_n216), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n249), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n252), .b(new_n263), .c(new_n262), .d(new_n236), .o1(new_n264));
  oaih12aa1n02x5               g169(.a(new_n254), .b(new_n264), .c(new_n261), .o1(new_n265));
  tech160nm_fiaoi012aa1n02p5x5 g170(.a(new_n259), .b(new_n265), .c(new_n257), .o1(new_n266));
  norp02aa1n03x5               g171(.a(new_n266), .b(new_n260), .o1(\s[28] ));
  norb02aa1n02x5               g172(.a(new_n254), .b(new_n259), .out0(new_n268));
  oaih12aa1n02x5               g173(.a(new_n268), .b(new_n264), .c(new_n261), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[28] ), .b(\b[27] ), .c(new_n257), .carry(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[28] ), .b(\a[29] ), .out0(new_n271));
  tech160nm_fiaoi012aa1n02p5x5 g176(.a(new_n271), .b(new_n269), .c(new_n270), .o1(new_n272));
  aobi12aa1n06x5               g177(.a(new_n268), .b(new_n253), .c(new_n251), .out0(new_n273));
  nano22aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n271), .out0(new_n274));
  norp02aa1n03x5               g179(.a(new_n272), .b(new_n274), .o1(\s[29] ));
  xorb03aa1n02x5               g180(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g181(.a(new_n254), .b(new_n271), .c(new_n259), .out0(new_n277));
  oaih12aa1n02x5               g182(.a(new_n277), .b(new_n264), .c(new_n261), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[29] ), .b(\b[28] ), .c(new_n270), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[29] ), .b(\a[30] ), .out0(new_n280));
  tech160nm_fiaoi012aa1n02p5x5 g185(.a(new_n280), .b(new_n278), .c(new_n279), .o1(new_n281));
  aobi12aa1n06x5               g186(.a(new_n277), .b(new_n253), .c(new_n251), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n282), .b(new_n279), .c(new_n280), .out0(new_n283));
  norp02aa1n03x5               g188(.a(new_n281), .b(new_n283), .o1(\s[30] ));
  xnrc02aa1n02x5               g189(.a(\b[30] ), .b(\a[31] ), .out0(new_n285));
  norb02aa1n03x5               g190(.a(new_n277), .b(new_n280), .out0(new_n286));
  aobi12aa1n06x5               g191(.a(new_n286), .b(new_n253), .c(new_n251), .out0(new_n287));
  oao003aa1n02x5               g192(.a(\a[30] ), .b(\b[29] ), .c(new_n279), .carry(new_n288));
  nano22aa1n03x5               g193(.a(new_n287), .b(new_n285), .c(new_n288), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n286), .b(new_n264), .c(new_n261), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n285), .b(new_n290), .c(new_n288), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n289), .o1(\s[31] ));
  xnrb03aa1n02x5               g197(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g198(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n294));
  xorb03aa1n02x5               g199(.a(new_n294), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g200(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norp02aa1n02x5               g201(.a(\b[4] ), .b(\a[5] ), .o1(new_n297));
  aoib12aa1n02x5               g202(.a(new_n297), .b(new_n109), .c(new_n116), .out0(new_n298));
  xnrb03aa1n02x5               g203(.a(new_n298), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g204(.a(new_n115), .b(new_n298), .out0(new_n300));
  nona23aa1n02x4               g205(.a(new_n300), .b(new_n113), .c(new_n112), .d(new_n119), .out0(new_n301));
  aboi22aa1n03x5               g206(.a(new_n119), .b(new_n300), .c(new_n122), .d(new_n113), .out0(new_n302));
  norb02aa1n02x5               g207(.a(new_n301), .b(new_n302), .out0(\s[7] ));
  norb02aa1n02x5               g208(.a(new_n111), .b(new_n110), .out0(new_n304));
  xnbna2aa1n03x5               g209(.a(new_n304), .b(new_n301), .c(new_n122), .out0(\s[8] ));
  xnbna2aa1n03x5               g210(.a(new_n144), .b(new_n118), .c(new_n125), .out0(\s[9] ));
endmodule


