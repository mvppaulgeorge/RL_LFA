// Benchmark "adder" written by ABC on Wed Jul 17 16:20:42 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n333, new_n334, new_n335, new_n338, new_n340, new_n341,
    new_n343, new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n03x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand02aa1n03x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  tech160nm_fiaoi012aa1n05x5   g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  inv000aa1n02x5               g006(.a(new_n101), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  tech160nm_finand02aa1n03p5x5 g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand22aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n02x5               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  aoi012aa1n06x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  inv040aa1n03x5               g013(.a(new_n108), .o1(new_n109));
  aoi012aa1n06x5               g014(.a(new_n109), .b(new_n107), .c(new_n102), .o1(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[4] ), .b(\a[5] ), .out0(new_n111));
  nor002aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanb02aa1n03x5               g018(.a(new_n112), .b(new_n113), .out0(new_n114));
  nor002aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nano23aa1n02x4               g023(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n119));
  nona22aa1n02x4               g024(.a(new_n119), .b(new_n111), .c(new_n114), .out0(new_n120));
  inv000aa1d42x5               g025(.a(\a[8] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[7] ), .o1(new_n122));
  inv020aa1n02x5               g027(.a(new_n115), .o1(new_n123));
  norp02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  aoai13aa1n03x5               g029(.a(new_n116), .b(new_n112), .c(new_n124), .d(new_n113), .o1(new_n125));
  nand22aa1n02x5               g030(.a(new_n125), .b(new_n123), .o1(new_n126));
  oaoi03aa1n09x5               g031(.a(new_n121), .b(new_n122), .c(new_n126), .o1(new_n127));
  oai012aa1n12x5               g032(.a(new_n127), .b(new_n110), .c(new_n120), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  aoi012aa1n02x5               g034(.a(new_n97), .b(new_n128), .c(new_n129), .o1(new_n130));
  xnrb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanb02aa1n03x5               g036(.a(new_n103), .b(new_n104), .out0(new_n132));
  inv000aa1d42x5               g037(.a(\a[3] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(\b[2] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n134), .b(new_n133), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(new_n135), .b(new_n106), .o1(new_n136));
  norp03aa1n02x5               g041(.a(new_n101), .b(new_n132), .c(new_n136), .o1(new_n137));
  nona23aa1n02x4               g042(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n138));
  norp03aa1n02x5               g043(.a(new_n138), .b(new_n114), .c(new_n111), .o1(new_n139));
  tech160nm_fioai012aa1n04x5   g044(.a(new_n139), .b(new_n137), .c(new_n109), .o1(new_n140));
  norp02aa1n03x5               g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  nand22aa1n03x5               g046(.a(\b[9] ), .b(\a[10] ), .o1(new_n142));
  nona23aa1n09x5               g047(.a(new_n129), .b(new_n142), .c(new_n141), .d(new_n97), .out0(new_n143));
  tech160nm_fioai012aa1n05x5   g048(.a(new_n142), .b(new_n97), .c(new_n141), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n140), .d(new_n127), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n03x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  nand42aa1n02x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n147), .b(new_n145), .c(new_n148), .o1(new_n149));
  xnrb03aa1n02x5               g054(.a(new_n149), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1n03x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nona23aa1n09x5               g057(.a(new_n152), .b(new_n148), .c(new_n147), .d(new_n151), .out0(new_n153));
  nor042aa1n02x5               g058(.a(new_n153), .b(new_n143), .o1(new_n154));
  inv000aa1n02x5               g059(.a(new_n154), .o1(new_n155));
  inv040aa1n03x5               g060(.a(new_n144), .o1(new_n156));
  nano23aa1n03x7               g061(.a(new_n147), .b(new_n151), .c(new_n152), .d(new_n148), .out0(new_n157));
  oaih12aa1n02x5               g062(.a(new_n152), .b(new_n151), .c(new_n147), .o1(new_n158));
  aobi12aa1n09x5               g063(.a(new_n158), .b(new_n157), .c(new_n156), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n155), .c(new_n140), .d(new_n127), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1n03x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n162), .b(new_n160), .c(new_n163), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fioai012aa1n04x5   g070(.a(new_n158), .b(new_n153), .c(new_n144), .o1(new_n166));
  nor042aa1n09x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand42aa1n06x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nona23aa1n12x5               g073(.a(new_n168), .b(new_n163), .c(new_n162), .d(new_n167), .out0(new_n169));
  inv040aa1n02x5               g074(.a(new_n169), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n166), .c(new_n128), .d(new_n154), .o1(new_n171));
  oaih12aa1n12x5               g076(.a(new_n168), .b(new_n167), .c(new_n162), .o1(new_n172));
  xnrc02aa1n12x5               g077(.a(\b[14] ), .b(\a[15] ), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n171), .c(new_n172), .out0(\s[15] ));
  tech160nm_fixnrc02aa1n04x5   g080(.a(\b[15] ), .b(\a[16] ), .out0(new_n176));
  nor042aa1n06x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n178), .b(new_n173), .c(new_n171), .d(new_n172), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n176), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n172), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n174), .b(new_n181), .c(new_n160), .d(new_n170), .o1(new_n182));
  nona22aa1n02x4               g087(.a(new_n182), .b(new_n176), .c(new_n177), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n183), .o1(\s[16] ));
  nor042aa1n04x5               g089(.a(new_n176), .b(new_n173), .o1(new_n185));
  nona23aa1n12x5               g090(.a(new_n170), .b(new_n185), .c(new_n153), .d(new_n143), .out0(new_n186));
  nor003aa1n03x5               g091(.a(new_n169), .b(new_n173), .c(new_n176), .o1(new_n187));
  oao003aa1n02x5               g092(.a(\a[16] ), .b(\b[15] ), .c(new_n178), .carry(new_n188));
  oai013aa1n03x4               g093(.a(new_n188), .b(new_n173), .c(new_n176), .d(new_n172), .o1(new_n189));
  aoi012aa1n06x5               g094(.a(new_n189), .b(new_n166), .c(new_n187), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n186), .c(new_n140), .d(new_n127), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g097(.a(\a[17] ), .o1(new_n193));
  nanb02aa1n02x5               g098(.a(\b[16] ), .b(new_n193), .out0(new_n194));
  inv040aa1n02x5               g099(.a(new_n186), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n185), .b(new_n170), .o1(new_n196));
  oabi12aa1n06x5               g101(.a(new_n189), .b(new_n159), .c(new_n196), .out0(new_n197));
  xorc02aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n197), .c(new_n128), .d(new_n195), .o1(new_n199));
  xnrc02aa1n02x5               g104(.a(\b[17] ), .b(\a[18] ), .out0(new_n200));
  xobna2aa1n03x5               g105(.a(new_n200), .b(new_n199), .c(new_n194), .out0(\s[18] ));
  inv000aa1d42x5               g106(.a(\a[18] ), .o1(new_n202));
  xroi22aa1d04x5               g107(.a(new_n193), .b(\b[16] ), .c(new_n202), .d(\b[17] ), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n197), .c(new_n128), .d(new_n195), .o1(new_n204));
  oai022aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  oaib12aa1n09x5               g110(.a(new_n205), .b(new_n202), .c(\b[17] ), .out0(new_n206));
  nor042aa1n04x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nanp02aa1n12x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nanb02aa1d24x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n204), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g117(.a(new_n204), .b(new_n206), .o1(new_n213));
  nor042aa1n04x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand42aa1n06x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nanb02aa1n06x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n207), .c(new_n213), .d(new_n210), .o1(new_n217));
  oaoi03aa1n02x5               g122(.a(\a[18] ), .b(\b[17] ), .c(new_n194), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n210), .b(new_n218), .c(new_n191), .d(new_n203), .o1(new_n219));
  nona22aa1n03x5               g124(.a(new_n219), .b(new_n216), .c(new_n207), .out0(new_n220));
  nanp02aa1n02x5               g125(.a(new_n217), .b(new_n220), .o1(\s[20] ));
  nand22aa1n03x5               g126(.a(new_n128), .b(new_n195), .o1(new_n222));
  nano23aa1d15x5               g127(.a(new_n207), .b(new_n214), .c(new_n215), .d(new_n208), .out0(new_n223));
  nanb03aa1n12x5               g128(.a(new_n200), .b(new_n223), .c(new_n198), .out0(new_n224));
  oai022aa1n02x5               g129(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n225));
  aoi022aa1n06x5               g130(.a(new_n223), .b(new_n218), .c(new_n215), .d(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n190), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xnrc02aa1n12x5               g134(.a(\b[20] ), .b(\a[21] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  tech160nm_fixnrc02aa1n04x5   g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n229), .c(new_n227), .d(new_n231), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n224), .o1(new_n234));
  oai012aa1n02x5               g139(.a(new_n215), .b(new_n214), .c(new_n207), .o1(new_n235));
  oai013aa1d12x5               g140(.a(new_n235), .b(new_n206), .c(new_n209), .d(new_n216), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n231), .b(new_n236), .c(new_n191), .d(new_n234), .o1(new_n237));
  nona22aa1n02x4               g142(.a(new_n237), .b(new_n232), .c(new_n229), .out0(new_n238));
  nanp02aa1n02x5               g143(.a(new_n233), .b(new_n238), .o1(\s[22] ));
  nor042aa1n09x5               g144(.a(new_n232), .b(new_n230), .o1(new_n240));
  nand23aa1d12x5               g145(.a(new_n203), .b(new_n240), .c(new_n223), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\a[22] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[21] ), .o1(new_n243));
  oao003aa1n02x5               g148(.a(new_n242), .b(new_n243), .c(new_n229), .carry(new_n244));
  aoi012aa1n12x5               g149(.a(new_n244), .b(new_n236), .c(new_n240), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n241), .c(new_n222), .d(new_n190), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n04x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  nand02aa1n03x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  nanb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  nor042aa1n04x5               g156(.a(\b[23] ), .b(\a[24] ), .o1(new_n252));
  nanp02aa1n04x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  nanb02aa1n02x5               g158(.a(new_n252), .b(new_n253), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n248), .c(new_n246), .d(new_n251), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n241), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n245), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n251), .b(new_n257), .c(new_n191), .d(new_n256), .o1(new_n258));
  nona22aa1n03x5               g163(.a(new_n258), .b(new_n254), .c(new_n248), .out0(new_n259));
  nanp02aa1n02x5               g164(.a(new_n255), .b(new_n259), .o1(\s[24] ));
  nano23aa1d18x5               g165(.a(new_n248), .b(new_n252), .c(new_n253), .d(new_n249), .out0(new_n261));
  nano22aa1d15x5               g166(.a(new_n224), .b(new_n240), .c(new_n261), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n197), .c(new_n128), .d(new_n195), .o1(new_n263));
  oaoi03aa1n02x5               g168(.a(new_n242), .b(new_n243), .c(new_n229), .o1(new_n264));
  nona23aa1n09x5               g169(.a(new_n253), .b(new_n249), .c(new_n248), .d(new_n252), .out0(new_n265));
  nanp02aa1n02x5               g170(.a(new_n248), .b(new_n253), .o1(new_n266));
  oai122aa1n06x5               g171(.a(new_n266), .b(new_n265), .c(new_n264), .d(\b[23] ), .e(\a[24] ), .o1(new_n267));
  aoi013aa1n09x5               g172(.a(new_n267), .b(new_n236), .c(new_n240), .d(new_n261), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n263), .c(new_n268), .out0(\s[25] ));
  nand42aa1n03x5               g175(.a(new_n263), .b(new_n268), .o1(new_n271));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  tech160nm_fixnrc02aa1n02p5x5 g177(.a(\b[25] ), .b(\a[26] ), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n272), .c(new_n271), .d(new_n269), .o1(new_n274));
  inv020aa1n02x5               g179(.a(new_n268), .o1(new_n275));
  aoai13aa1n03x5               g180(.a(new_n269), .b(new_n275), .c(new_n191), .d(new_n262), .o1(new_n276));
  nona22aa1n02x4               g181(.a(new_n276), .b(new_n273), .c(new_n272), .out0(new_n277));
  nanp02aa1n03x5               g182(.a(new_n274), .b(new_n277), .o1(\s[26] ));
  norb02aa1n06x5               g183(.a(new_n269), .b(new_n273), .out0(new_n279));
  nano22aa1d15x5               g184(.a(new_n241), .b(new_n261), .c(new_n279), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n197), .c(new_n128), .d(new_n195), .o1(new_n281));
  nano22aa1n03x7               g186(.a(new_n226), .b(new_n240), .c(new_n261), .out0(new_n282));
  inv000aa1d42x5               g187(.a(\a[26] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(\b[25] ), .o1(new_n284));
  oao003aa1n02x5               g189(.a(new_n283), .b(new_n284), .c(new_n272), .carry(new_n285));
  oaoi13aa1n09x5               g190(.a(new_n285), .b(new_n279), .c(new_n282), .d(new_n267), .o1(new_n286));
  xorc02aa1n02x5               g191(.a(\a[27] ), .b(\b[26] ), .out0(new_n287));
  xnbna2aa1n03x5               g192(.a(new_n287), .b(new_n281), .c(new_n286), .out0(\s[27] ));
  tech160nm_finand02aa1n03p5x5 g193(.a(new_n281), .b(new_n286), .o1(new_n289));
  norp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  norp02aa1n02x5               g195(.a(\b[27] ), .b(\a[28] ), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(\b[27] ), .b(\a[28] ), .o1(new_n292));
  norb02aa1n06x4               g197(.a(new_n292), .b(new_n291), .out0(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n290), .c(new_n289), .d(new_n287), .o1(new_n295));
  aoi122aa1n06x5               g200(.a(new_n252), .b(new_n253), .c(new_n248), .d(new_n261), .e(new_n244), .o1(new_n296));
  nona32aa1n06x5               g201(.a(new_n236), .b(new_n265), .c(new_n232), .d(new_n230), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n279), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n285), .o1(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n298), .c(new_n297), .d(new_n296), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n287), .b(new_n300), .c(new_n191), .d(new_n280), .o1(new_n301));
  nona22aa1n03x5               g206(.a(new_n301), .b(new_n294), .c(new_n290), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n295), .b(new_n302), .o1(\s[28] ));
  norb02aa1n09x5               g208(.a(new_n287), .b(new_n294), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n300), .c(new_n191), .d(new_n280), .o1(new_n305));
  oai012aa1n02x5               g210(.a(new_n292), .b(new_n291), .c(new_n290), .o1(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[28] ), .b(\a[29] ), .out0(new_n307));
  aoi012aa1n03x5               g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n304), .o1(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n309), .b(new_n281), .c(new_n286), .o1(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n306), .c(new_n307), .out0(new_n311));
  nor002aa1n02x5               g216(.a(new_n308), .b(new_n311), .o1(\s[29] ));
  xorb03aa1n02x5               g217(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g218(.a(new_n307), .b(new_n287), .c(new_n293), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n300), .c(new_n191), .d(new_n280), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .carry(new_n316));
  xnrc02aa1n02x5               g221(.a(\b[29] ), .b(\a[30] ), .out0(new_n317));
  aoi012aa1n03x5               g222(.a(new_n317), .b(new_n315), .c(new_n316), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n314), .o1(new_n319));
  tech160nm_fiaoi012aa1n02p5x5 g224(.a(new_n319), .b(new_n281), .c(new_n286), .o1(new_n320));
  nano22aa1n03x5               g225(.a(new_n320), .b(new_n316), .c(new_n317), .out0(new_n321));
  nor002aa1n02x5               g226(.a(new_n318), .b(new_n321), .o1(\s[30] ));
  xnrc02aa1n02x5               g227(.a(\b[30] ), .b(\a[31] ), .out0(new_n323));
  nano23aa1n06x5               g228(.a(new_n317), .b(new_n307), .c(new_n287), .d(new_n293), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n300), .c(new_n191), .d(new_n280), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .c(new_n316), .carry(new_n326));
  aoi012aa1n03x5               g231(.a(new_n323), .b(new_n325), .c(new_n326), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n324), .o1(new_n328));
  tech160nm_fiaoi012aa1n02p5x5 g233(.a(new_n328), .b(new_n281), .c(new_n286), .o1(new_n329));
  nano22aa1n03x5               g234(.a(new_n329), .b(new_n323), .c(new_n326), .out0(new_n330));
  norp02aa1n03x5               g235(.a(new_n327), .b(new_n330), .o1(\s[31] ));
  xnbna2aa1n03x5               g236(.a(new_n101), .b(new_n106), .c(new_n135), .out0(\s[3] ));
  oai013aa1n03x5               g237(.a(new_n108), .b(new_n101), .c(new_n132), .d(new_n136), .o1(new_n333));
  aboi22aa1n03x5               g238(.a(new_n103), .b(new_n104), .c(new_n133), .d(new_n134), .out0(new_n334));
  oa0012aa1n02x5               g239(.a(new_n334), .b(new_n101), .c(new_n136), .o(new_n335));
  oaoi13aa1n02x5               g240(.a(new_n335), .b(new_n333), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g241(.a(new_n333), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoib12aa1n02x5               g242(.a(new_n124), .b(new_n333), .c(new_n111), .out0(new_n338));
  xnrb03aa1n02x5               g243(.a(new_n338), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g244(.a(new_n116), .b(new_n115), .out0(new_n340));
  nanb02aa1n03x5               g245(.a(new_n114), .b(new_n338), .out0(new_n341));
  xobna2aa1n03x5               g246(.a(new_n340), .b(new_n341), .c(new_n113), .out0(\s[7] ));
  norb02aa1n02x5               g247(.a(new_n118), .b(new_n117), .out0(new_n343));
  nanp03aa1n02x5               g248(.a(new_n341), .b(new_n113), .c(new_n340), .o1(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n343), .b(new_n344), .c(new_n123), .out0(\s[8] ));
  xorb03aa1n02x5               g250(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


