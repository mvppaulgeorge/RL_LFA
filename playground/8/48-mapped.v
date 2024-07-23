// Benchmark "adder" written by ABC on Wed Jul 17 16:29:28 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n334, new_n335, new_n338, new_n339, new_n341,
    new_n342, new_n343, new_n345;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand02aa1n03x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  tech160nm_fiaoi012aa1n05x5   g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  xorc02aa1n02x5               g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  nor042aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n03x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nanb03aa1n12x5               g010(.a(new_n101), .b(new_n102), .c(new_n105), .out0(new_n106));
  inv000aa1d42x5               g011(.a(\a[4] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[3] ), .o1(new_n108));
  oaoi03aa1n12x5               g013(.a(new_n107), .b(new_n108), .c(new_n103), .o1(new_n109));
  tech160nm_fixnrc02aa1n04x5   g014(.a(\b[4] ), .b(\a[5] ), .out0(new_n110));
  nor042aa1n06x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nanp02aa1n12x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nanb02aa1n02x5               g017(.a(new_n111), .b(new_n112), .out0(new_n113));
  nor002aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nor022aa1n06x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nano23aa1n02x4               g022(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n118));
  nona22aa1n02x4               g023(.a(new_n118), .b(new_n110), .c(new_n113), .out0(new_n119));
  inv000aa1d42x5               g024(.a(\a[7] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[5] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[4] ), .o1(new_n122));
  aoai13aa1n06x5               g027(.a(new_n112), .b(new_n111), .c(new_n121), .d(new_n122), .o1(new_n123));
  oaib12aa1n02x5               g028(.a(new_n123), .b(\b[6] ), .c(new_n120), .out0(new_n124));
  aoi022aa1n02x5               g029(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n125));
  aoi012aa1n12x5               g030(.a(new_n116), .b(new_n124), .c(new_n125), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n119), .c(new_n106), .d(new_n109), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  aoi012aa1n02x5               g033(.a(new_n97), .b(new_n127), .c(new_n128), .o1(new_n129));
  xnrb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g035(.a(new_n108), .b(new_n107), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[3] ), .b(\a[4] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(new_n131), .b(new_n132), .o1(new_n133));
  inv040aa1d32x5               g038(.a(\a[3] ), .o1(new_n134));
  inv000aa1d42x5               g039(.a(\b[2] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(new_n135), .b(new_n134), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(new_n136), .b(new_n104), .o1(new_n137));
  nor003aa1n02x5               g042(.a(new_n101), .b(new_n133), .c(new_n137), .o1(new_n138));
  inv000aa1n02x5               g043(.a(new_n109), .o1(new_n139));
  nona23aa1n02x4               g044(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n140));
  norp03aa1n02x5               g045(.a(new_n140), .b(new_n113), .c(new_n110), .o1(new_n141));
  oai012aa1n03x5               g046(.a(new_n141), .b(new_n138), .c(new_n139), .o1(new_n142));
  nor022aa1n04x5               g047(.a(\b[9] ), .b(\a[10] ), .o1(new_n143));
  nand42aa1n02x5               g048(.a(\b[9] ), .b(\a[10] ), .o1(new_n144));
  nona23aa1n09x5               g049(.a(new_n128), .b(new_n144), .c(new_n143), .d(new_n97), .out0(new_n145));
  oai012aa1n12x5               g050(.a(new_n144), .b(new_n97), .c(new_n143), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n142), .d(new_n126), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n04x5               g053(.a(\b[10] ), .b(\a[11] ), .o1(new_n149));
  nand42aa1n02x5               g054(.a(\b[10] ), .b(\a[11] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1n03x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nona23aa1n09x5               g059(.a(new_n154), .b(new_n150), .c(new_n149), .d(new_n153), .out0(new_n155));
  nor042aa1n03x5               g060(.a(new_n155), .b(new_n145), .o1(new_n156));
  inv000aa1n02x5               g061(.a(new_n156), .o1(new_n157));
  inv000aa1n02x5               g062(.a(new_n146), .o1(new_n158));
  nano23aa1n02x4               g063(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n159));
  oai012aa1n02x5               g064(.a(new_n154), .b(new_n153), .c(new_n149), .o1(new_n160));
  aobi12aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n158), .out0(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n157), .c(new_n142), .d(new_n126), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n09x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  aoi012aa1n03x5               g070(.a(new_n164), .b(new_n162), .c(new_n165), .o1(new_n166));
  xnrb03aa1n02x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fioai012aa1n03p5x5 g072(.a(new_n160), .b(new_n155), .c(new_n146), .o1(new_n168));
  nor002aa1d32x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand02aa1d04x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nona23aa1n09x5               g075(.a(new_n170), .b(new_n165), .c(new_n164), .d(new_n169), .out0(new_n171));
  inv040aa1n02x5               g076(.a(new_n171), .o1(new_n172));
  aoai13aa1n03x5               g077(.a(new_n172), .b(new_n168), .c(new_n127), .d(new_n156), .o1(new_n173));
  oaih12aa1n12x5               g078(.a(new_n170), .b(new_n169), .c(new_n164), .o1(new_n174));
  xnrc02aa1n12x5               g079(.a(\b[14] ), .b(\a[15] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n173), .c(new_n174), .out0(\s[15] ));
  xnrc02aa1n12x5               g082(.a(\b[15] ), .b(\a[16] ), .out0(new_n178));
  nor042aa1n06x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  aoai13aa1n03x5               g085(.a(new_n180), .b(new_n175), .c(new_n173), .d(new_n174), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n181), .b(new_n178), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n174), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n176), .b(new_n183), .c(new_n162), .d(new_n172), .o1(new_n184));
  nona22aa1n02x4               g089(.a(new_n184), .b(new_n178), .c(new_n179), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n182), .b(new_n185), .o1(\s[16] ));
  nor042aa1n06x5               g091(.a(new_n178), .b(new_n175), .o1(new_n187));
  nona23aa1n09x5               g092(.a(new_n172), .b(new_n187), .c(new_n155), .d(new_n145), .out0(new_n188));
  norp03aa1n06x5               g093(.a(new_n171), .b(new_n175), .c(new_n178), .o1(new_n189));
  oao003aa1n02x5               g094(.a(\a[16] ), .b(\b[15] ), .c(new_n180), .carry(new_n190));
  oai013aa1n09x5               g095(.a(new_n190), .b(new_n175), .c(new_n178), .d(new_n174), .o1(new_n191));
  aoi012aa1n09x5               g096(.a(new_n191), .b(new_n168), .c(new_n189), .o1(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n188), .c(new_n142), .d(new_n126), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(\b[16] ), .b(new_n195), .out0(new_n196));
  inv040aa1n03x5               g101(.a(new_n188), .o1(new_n197));
  nanp02aa1n04x5               g102(.a(new_n187), .b(new_n172), .o1(new_n198));
  oabi12aa1n06x5               g103(.a(new_n191), .b(new_n161), .c(new_n198), .out0(new_n199));
  tech160nm_fixorc02aa1n03p5x5 g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  aoai13aa1n02x5               g105(.a(new_n200), .b(new_n199), .c(new_n127), .d(new_n197), .o1(new_n201));
  xnrc02aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .out0(new_n202));
  xobna2aa1n03x5               g107(.a(new_n202), .b(new_n201), .c(new_n196), .out0(\s[18] ));
  inv000aa1d42x5               g108(.a(\a[18] ), .o1(new_n204));
  xroi22aa1d06x4               g109(.a(new_n195), .b(\b[16] ), .c(new_n204), .d(\b[17] ), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n199), .c(new_n127), .d(new_n197), .o1(new_n206));
  oai022aa1n02x5               g111(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n207));
  oaib12aa1n09x5               g112(.a(new_n207), .b(new_n204), .c(\b[17] ), .out0(new_n208));
  norp02aa1n09x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand42aa1n08x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanb02aa1d24x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n206), .c(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n03x5               g119(.a(new_n206), .b(new_n208), .o1(new_n215));
  nor042aa1n04x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand42aa1n04x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanb02aa1n06x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n209), .c(new_n215), .d(new_n212), .o1(new_n219));
  oaoi03aa1n02x5               g124(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n212), .b(new_n220), .c(new_n193), .d(new_n205), .o1(new_n221));
  nona22aa1n02x4               g126(.a(new_n221), .b(new_n218), .c(new_n209), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(new_n219), .b(new_n222), .o1(\s[20] ));
  nanp02aa1n06x5               g128(.a(new_n127), .b(new_n197), .o1(new_n224));
  nano23aa1d15x5               g129(.a(new_n209), .b(new_n216), .c(new_n217), .d(new_n210), .out0(new_n225));
  nanb03aa1d18x5               g130(.a(new_n202), .b(new_n225), .c(new_n200), .out0(new_n226));
  oai022aa1n02x5               g131(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n227));
  aoi022aa1n06x5               g132(.a(new_n225), .b(new_n220), .c(new_n217), .d(new_n227), .o1(new_n228));
  aoai13aa1n04x5               g133(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n192), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n04x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  norp02aa1n04x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nand02aa1n03x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nanb02aa1n06x5               g140(.a(new_n234), .b(new_n235), .out0(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n231), .c(new_n229), .d(new_n233), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n226), .o1(new_n238));
  oai012aa1n02x5               g143(.a(new_n217), .b(new_n216), .c(new_n209), .o1(new_n239));
  oai013aa1d12x5               g144(.a(new_n239), .b(new_n208), .c(new_n211), .d(new_n218), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n233), .b(new_n240), .c(new_n193), .d(new_n238), .o1(new_n241));
  nona22aa1n02x4               g146(.a(new_n241), .b(new_n236), .c(new_n231), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n237), .b(new_n242), .o1(\s[22] ));
  nor042aa1n09x5               g148(.a(new_n232), .b(new_n236), .o1(new_n244));
  nand23aa1n09x5               g149(.a(new_n205), .b(new_n244), .c(new_n225), .o1(new_n245));
  oai012aa1n02x5               g150(.a(new_n235), .b(new_n234), .c(new_n231), .o1(new_n246));
  aobi12aa1n18x5               g151(.a(new_n246), .b(new_n240), .c(new_n244), .out0(new_n247));
  aoai13aa1n04x5               g152(.a(new_n247), .b(new_n245), .c(new_n224), .d(new_n192), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  nanp02aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  nanb02aa1n02x5               g156(.a(new_n250), .b(new_n251), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  nor042aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  nanp02aa1n03x5               g159(.a(\b[23] ), .b(\a[24] ), .o1(new_n255));
  nanb02aa1n02x5               g160(.a(new_n254), .b(new_n255), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n250), .c(new_n248), .d(new_n253), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n245), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n247), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n253), .b(new_n259), .c(new_n193), .d(new_n258), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n256), .c(new_n250), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n257), .b(new_n261), .o1(\s[24] ));
  nona23aa1d18x5               g167(.a(new_n255), .b(new_n251), .c(new_n250), .d(new_n254), .out0(new_n263));
  inv040aa1n08x5               g168(.a(new_n263), .o1(new_n264));
  nano22aa1n06x5               g169(.a(new_n226), .b(new_n244), .c(new_n264), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n199), .c(new_n127), .d(new_n197), .o1(new_n266));
  oai012aa1n02x5               g171(.a(new_n255), .b(new_n254), .c(new_n250), .o1(new_n267));
  tech160nm_fioai012aa1n03p5x5 g172(.a(new_n267), .b(new_n263), .c(new_n246), .o1(new_n268));
  aoi013aa1n09x5               g173(.a(new_n268), .b(new_n240), .c(new_n244), .d(new_n264), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n266), .c(new_n269), .out0(\s[25] ));
  nand42aa1n03x5               g176(.a(new_n266), .b(new_n269), .o1(new_n272));
  norp02aa1n02x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[25] ), .b(\a[26] ), .out0(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n272), .d(new_n270), .o1(new_n275));
  inv020aa1n02x5               g180(.a(new_n269), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n270), .b(new_n276), .c(new_n193), .d(new_n265), .o1(new_n277));
  nona22aa1n02x4               g182(.a(new_n277), .b(new_n274), .c(new_n273), .out0(new_n278));
  nanp02aa1n03x5               g183(.a(new_n275), .b(new_n278), .o1(\s[26] ));
  norb02aa1n06x5               g184(.a(new_n270), .b(new_n274), .out0(new_n280));
  nano22aa1d15x5               g185(.a(new_n245), .b(new_n264), .c(new_n280), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n199), .c(new_n127), .d(new_n197), .o1(new_n282));
  nano22aa1n03x5               g187(.a(new_n228), .b(new_n244), .c(new_n264), .out0(new_n283));
  inv000aa1d42x5               g188(.a(\a[26] ), .o1(new_n284));
  inv000aa1d42x5               g189(.a(\b[25] ), .o1(new_n285));
  oao003aa1n02x5               g190(.a(new_n284), .b(new_n285), .c(new_n273), .carry(new_n286));
  oaoi13aa1n09x5               g191(.a(new_n286), .b(new_n280), .c(new_n283), .d(new_n268), .o1(new_n287));
  xorc02aa1n12x5               g192(.a(\a[27] ), .b(\b[26] ), .out0(new_n288));
  xnbna2aa1n03x5               g193(.a(new_n288), .b(new_n282), .c(new_n287), .out0(\s[27] ));
  tech160nm_fixorc02aa1n05x5   g194(.a(\a[28] ), .b(\b[27] ), .out0(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  norp02aa1n02x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  inv000aa1n03x5               g197(.a(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n288), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n293), .b(new_n294), .c(new_n282), .d(new_n287), .o1(new_n295));
  nand02aa1n02x5               g200(.a(new_n295), .b(new_n291), .o1(new_n296));
  nona32aa1n03x5               g201(.a(new_n240), .b(new_n263), .c(new_n236), .d(new_n232), .out0(new_n297));
  inv030aa1n02x5               g202(.a(new_n268), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n280), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n286), .o1(new_n300));
  aoai13aa1n06x5               g205(.a(new_n300), .b(new_n299), .c(new_n297), .d(new_n298), .o1(new_n301));
  aoai13aa1n02x5               g206(.a(new_n288), .b(new_n301), .c(new_n193), .d(new_n281), .o1(new_n302));
  nona22aa1n03x5               g207(.a(new_n302), .b(new_n291), .c(new_n292), .out0(new_n303));
  nanp02aa1n03x5               g208(.a(new_n296), .b(new_n303), .o1(\s[28] ));
  and002aa1n06x5               g209(.a(new_n290), .b(new_n288), .o(new_n305));
  aoai13aa1n02x7               g210(.a(new_n305), .b(new_n301), .c(new_n193), .d(new_n281), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[28] ), .b(\a[29] ), .out0(new_n308));
  aoi012aa1n03x5               g213(.a(new_n308), .b(new_n306), .c(new_n307), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n305), .o1(new_n310));
  aoi012aa1n03x5               g215(.a(new_n310), .b(new_n282), .c(new_n287), .o1(new_n311));
  nano22aa1n03x5               g216(.a(new_n311), .b(new_n307), .c(new_n308), .out0(new_n312));
  nor002aa1n02x5               g217(.a(new_n309), .b(new_n312), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n12x5               g219(.a(new_n308), .b(new_n288), .c(new_n290), .out0(new_n315));
  aoai13aa1n02x7               g220(.a(new_n315), .b(new_n301), .c(new_n193), .d(new_n281), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .carry(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .out0(new_n318));
  aoi012aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n317), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n315), .o1(new_n320));
  aoi012aa1n03x5               g225(.a(new_n320), .b(new_n282), .c(new_n287), .o1(new_n321));
  nano22aa1n03x5               g226(.a(new_n321), .b(new_n317), .c(new_n318), .out0(new_n322));
  norp02aa1n03x5               g227(.a(new_n319), .b(new_n322), .o1(\s[30] ));
  nano23aa1n06x5               g228(.a(new_n318), .b(new_n308), .c(new_n290), .d(new_n288), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n301), .c(new_n193), .d(new_n281), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n326));
  xnrc02aa1n02x5               g231(.a(\b[30] ), .b(\a[31] ), .out0(new_n327));
  aoi012aa1n03x5               g232(.a(new_n327), .b(new_n325), .c(new_n326), .o1(new_n328));
  inv000aa1d42x5               g233(.a(new_n324), .o1(new_n329));
  aoi012aa1n02x7               g234(.a(new_n329), .b(new_n282), .c(new_n287), .o1(new_n330));
  nano22aa1n03x5               g235(.a(new_n330), .b(new_n326), .c(new_n327), .out0(new_n331));
  norp02aa1n03x5               g236(.a(new_n328), .b(new_n331), .o1(\s[31] ));
  xnbna2aa1n03x5               g237(.a(new_n101), .b(new_n104), .c(new_n136), .out0(\s[3] ));
  nanp02aa1n02x5               g238(.a(new_n133), .b(new_n136), .o1(new_n334));
  oab012aa1n02x4               g239(.a(new_n334), .b(new_n101), .c(new_n137), .out0(new_n335));
  oaoi13aa1n02x5               g240(.a(new_n335), .b(new_n131), .c(new_n138), .d(new_n139), .o1(\s[4] ));
  xobna2aa1n03x5               g241(.a(new_n110), .b(new_n106), .c(new_n109), .out0(\s[5] ));
  nanp02aa1n02x5               g242(.a(new_n122), .b(new_n121), .o1(new_n338));
  aoai13aa1n06x5               g243(.a(new_n338), .b(new_n110), .c(new_n106), .d(new_n109), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g245(.a(new_n115), .b(new_n114), .out0(new_n341));
  oaoi13aa1n02x5               g246(.a(new_n341), .b(new_n112), .c(new_n339), .d(new_n111), .o1(new_n342));
  oai112aa1n02x5               g247(.a(new_n341), .b(new_n112), .c(new_n339), .d(new_n111), .o1(new_n343));
  norb02aa1n02x5               g248(.a(new_n343), .b(new_n342), .out0(\s[7] ));
  oaib12aa1n02x5               g249(.a(new_n343), .b(\b[6] ), .c(new_n120), .out0(new_n345));
  xorb03aa1n02x5               g250(.a(new_n345), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g251(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


