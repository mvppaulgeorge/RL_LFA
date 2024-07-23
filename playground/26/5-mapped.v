// Benchmark "adder" written by ABC on Thu Jul 18 01:17:43 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n315, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nanp02aa1n09x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nano23aa1n03x7               g007(.a(new_n99), .b(new_n101), .c(new_n102), .d(new_n100), .out0(new_n103));
  nor042aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nand42aa1n06x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor042aa1n03x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  tech160nm_fiaoi012aa1n05x5   g011(.a(new_n104), .b(new_n106), .c(new_n105), .o1(new_n107));
  inv020aa1n02x5               g012(.a(new_n107), .o1(new_n108));
  tech160nm_fiao0012aa1n02p5x5 g013(.a(new_n99), .b(new_n101), .c(new_n100), .o(new_n109));
  tech160nm_fiaoi012aa1n03p5x5 g014(.a(new_n109), .b(new_n103), .c(new_n108), .o1(new_n110));
  nor042aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  aoi012aa1n12x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  inv000aa1n02x5               g019(.a(new_n114), .o1(new_n115));
  nor002aa1n06x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nand02aa1n04x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  nor022aa1n06x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nano23aa1n06x5               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nand42aa1n03x5               g025(.a(new_n120), .b(new_n115), .o1(new_n121));
  tech160nm_fiaoi012aa1n04x5   g026(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n122));
  nand42aa1n03x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  nano23aa1n03x5               g028(.a(new_n104), .b(new_n106), .c(new_n123), .d(new_n105), .out0(new_n124));
  nand42aa1n02x5               g029(.a(new_n124), .b(new_n103), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n110), .b(new_n125), .c(new_n121), .d(new_n122), .o1(new_n126));
  xnrc02aa1n12x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  aoi012aa1n02x5               g033(.a(new_n98), .b(new_n126), .c(new_n128), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .out0(new_n131));
  nor042aa1n02x5               g036(.a(new_n131), .b(new_n127), .o1(new_n132));
  aob012aa1n02x5               g037(.a(new_n98), .b(\b[9] ), .c(\a[10] ), .out0(new_n133));
  oaib12aa1n06x5               g038(.a(new_n133), .b(\b[9] ), .c(new_n97), .out0(new_n134));
  nor042aa1n09x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoai13aa1n06x5               g042(.a(new_n137), .b(new_n134), .c(new_n126), .d(new_n132), .o1(new_n138));
  aoi112aa1n02x5               g043(.a(new_n137), .b(new_n134), .c(new_n126), .d(new_n132), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  inv000aa1d42x5               g045(.a(new_n135), .o1(new_n141));
  nor042aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(new_n144));
  xobna2aa1n03x5               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  nano23aa1n03x7               g050(.a(new_n135), .b(new_n142), .c(new_n143), .d(new_n136), .out0(new_n146));
  nanp02aa1n04x5               g051(.a(new_n146), .b(new_n134), .o1(new_n147));
  aoi012aa1n02x5               g052(.a(new_n142), .b(new_n135), .c(new_n143), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n147), .b(new_n148), .o1(new_n149));
  nano22aa1n02x4               g054(.a(new_n131), .b(new_n146), .c(new_n128), .out0(new_n150));
  tech160nm_fiaoi012aa1n05x5   g055(.a(new_n149), .b(new_n126), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  oaoi03aa1n02x5               g057(.a(\a[13] ), .b(\b[12] ), .c(new_n151), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xorc02aa1n12x5               g059(.a(\a[15] ), .b(\b[14] ), .out0(new_n155));
  nor002aa1n03x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1n16x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nor042aa1n04x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n12x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nano23aa1n03x7               g064(.a(new_n156), .b(new_n158), .c(new_n159), .d(new_n157), .out0(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n149), .c(new_n126), .d(new_n150), .o1(new_n161));
  tech160nm_fioai012aa1n02p5x5 g066(.a(new_n159), .b(new_n158), .c(new_n156), .o1(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n155), .b(new_n161), .c(new_n162), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand42aa1n03x5               g069(.a(new_n161), .b(new_n162), .o1(new_n165));
  xorc02aa1n12x5               g070(.a(\a[16] ), .b(\b[15] ), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n164), .c(new_n165), .d(new_n155), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(new_n165), .b(new_n155), .o1(new_n169));
  nona22aa1n02x4               g074(.a(new_n169), .b(new_n167), .c(new_n164), .out0(new_n170));
  nanp02aa1n02x5               g075(.a(new_n170), .b(new_n168), .o1(\s[16] ));
  inv020aa1n03x5               g076(.a(new_n162), .o1(new_n172));
  orn002aa1n02x5               g077(.a(\a[15] ), .b(\b[14] ), .o(new_n173));
  oaoi03aa1n02x5               g078(.a(\a[16] ), .b(\b[15] ), .c(new_n173), .o1(new_n174));
  aoi013aa1n03x5               g079(.a(new_n174), .b(new_n172), .c(new_n166), .d(new_n155), .o1(new_n175));
  nand03aa1n04x5               g080(.a(new_n160), .b(new_n155), .c(new_n166), .o1(new_n176));
  tech160nm_fiaoi012aa1n04x5   g081(.a(new_n176), .b(new_n147), .c(new_n148), .o1(new_n177));
  norb02aa1n09x5               g082(.a(new_n175), .b(new_n177), .out0(new_n178));
  nona23aa1n02x4               g083(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n179));
  oabi12aa1n02x5               g084(.a(new_n109), .b(new_n179), .c(new_n107), .out0(new_n180));
  nona23aa1n03x5               g085(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n181));
  oai012aa1n04x7               g086(.a(new_n122), .b(new_n181), .c(new_n114), .o1(new_n182));
  inv000aa1n04x5               g087(.a(new_n125), .o1(new_n183));
  nano22aa1n03x7               g088(.a(new_n176), .b(new_n132), .c(new_n146), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n180), .c(new_n183), .d(new_n182), .o1(new_n185));
  nanp02aa1n06x5               g090(.a(new_n185), .b(new_n178), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g092(.a(\a[18] ), .o1(new_n188));
  inv040aa1d30x5               g093(.a(\a[17] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[16] ), .o1(new_n190));
  oaoi03aa1n03x5               g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  xroi22aa1d06x4               g097(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  nor042aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  aoi112aa1n09x5               g100(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n196));
  nor002aa1n04x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  aoai13aa1n04x5               g102(.a(new_n197), .b(new_n194), .c(new_n185), .d(new_n178), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nand02aa1d08x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nor002aa1d32x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand02aa1d28x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n15x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n206), .b(new_n201), .c(new_n198), .d(new_n202), .o1(new_n207));
  norb02aa1n06x4               g112(.a(new_n202), .b(new_n201), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n198), .b(new_n208), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n209), .b(new_n206), .c(new_n201), .out0(new_n210));
  nanp02aa1n03x5               g115(.a(new_n210), .b(new_n207), .o1(\s[20] ));
  nona23aa1d18x5               g116(.a(new_n204), .b(new_n202), .c(new_n201), .d(new_n203), .out0(new_n212));
  aoi012aa1n06x5               g117(.a(new_n203), .b(new_n201), .c(new_n204), .o1(new_n213));
  oai012aa1n12x5               g118(.a(new_n213), .b(new_n212), .c(new_n197), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nanb02aa1n06x5               g120(.a(new_n212), .b(new_n193), .out0(new_n216));
  aoai13aa1n04x5               g121(.a(new_n215), .b(new_n216), .c(new_n185), .d(new_n178), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n08x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n12x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[21] ), .b(\a[22] ), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(new_n217), .b(new_n220), .o1(new_n223));
  nona22aa1n02x4               g128(.a(new_n223), .b(new_n221), .c(new_n219), .out0(new_n224));
  nanp02aa1n03x5               g129(.a(new_n224), .b(new_n222), .o1(\s[22] ));
  oai112aa1n06x5               g130(.a(new_n208), .b(new_n205), .c(new_n196), .d(new_n195), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(new_n221), .b(new_n220), .out0(new_n227));
  inv000aa1d42x5               g132(.a(\a[22] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\b[21] ), .o1(new_n229));
  oaoi03aa1n12x5               g134(.a(new_n228), .b(new_n229), .c(new_n219), .o1(new_n230));
  aoai13aa1n12x5               g135(.a(new_n230), .b(new_n227), .c(new_n226), .d(new_n213), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  nona23aa1n08x5               g137(.a(new_n193), .b(new_n220), .c(new_n221), .d(new_n212), .out0(new_n233));
  aoai13aa1n04x5               g138(.a(new_n232), .b(new_n233), .c(new_n185), .d(new_n178), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  tech160nm_fixorc02aa1n03p5x5 g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xorc02aa1n12x5               g142(.a(\a[24] ), .b(\b[23] ), .out0(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(new_n234), .b(new_n237), .o1(new_n241));
  nona22aa1n02x5               g146(.a(new_n241), .b(new_n239), .c(new_n236), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n242), .b(new_n240), .o1(\s[24] ));
  aoai13aa1n06x5               g148(.a(new_n175), .b(new_n176), .c(new_n148), .d(new_n147), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n220), .b(new_n221), .out0(new_n245));
  and002aa1n02x5               g150(.a(new_n238), .b(new_n237), .o(new_n246));
  nano22aa1n03x7               g151(.a(new_n216), .b(new_n246), .c(new_n245), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n244), .c(new_n126), .d(new_n184), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n230), .o1(new_n249));
  aoai13aa1n02x7               g154(.a(new_n246), .b(new_n249), .c(new_n214), .d(new_n245), .o1(new_n250));
  inv000aa1d42x5               g155(.a(\a[24] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(\b[23] ), .o1(new_n252));
  oaoi03aa1n12x5               g157(.a(new_n251), .b(new_n252), .c(new_n236), .o1(new_n253));
  nand02aa1n02x5               g158(.a(new_n250), .b(new_n253), .o1(new_n254));
  nanb02aa1n03x5               g159(.a(new_n254), .b(new_n248), .out0(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  tech160nm_fixorc02aa1n03p5x5 g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  xorc02aa1n12x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n257), .c(new_n255), .d(new_n258), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n258), .b(new_n254), .c(new_n186), .d(new_n247), .o1(new_n262));
  nona22aa1n02x5               g167(.a(new_n262), .b(new_n260), .c(new_n257), .out0(new_n263));
  nanp02aa1n03x5               g168(.a(new_n261), .b(new_n263), .o1(\s[26] ));
  inv000aa1d42x5               g169(.a(new_n253), .o1(new_n265));
  and002aa1n06x5               g170(.a(new_n259), .b(new_n258), .o(new_n266));
  aoai13aa1n04x5               g171(.a(new_n266), .b(new_n265), .c(new_n231), .d(new_n246), .o1(new_n267));
  aoi112aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n268));
  oab012aa1n02x4               g173(.a(new_n268), .b(\a[26] ), .c(\b[25] ), .out0(new_n269));
  nano22aa1n03x7               g174(.a(new_n233), .b(new_n246), .c(new_n266), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n244), .c(new_n126), .d(new_n184), .o1(new_n271));
  nand43aa1n03x5               g176(.a(new_n271), .b(new_n267), .c(new_n269), .o1(new_n272));
  xorb03aa1n03x5               g177(.a(new_n272), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  xorc02aa1n02x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  aoai13aa1n03x5               g181(.a(new_n276), .b(new_n274), .c(new_n272), .d(new_n275), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n266), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n269), .b(new_n278), .c(new_n250), .d(new_n253), .o1(new_n279));
  aobi12aa1n06x5               g184(.a(new_n270), .b(new_n185), .c(new_n178), .out0(new_n280));
  oaih12aa1n02x5               g185(.a(new_n275), .b(new_n279), .c(new_n280), .o1(new_n281));
  nona22aa1n02x5               g186(.a(new_n281), .b(new_n276), .c(new_n274), .out0(new_n282));
  nanp02aa1n03x5               g187(.a(new_n277), .b(new_n282), .o1(\s[28] ));
  norb02aa1n02x5               g188(.a(new_n275), .b(new_n276), .out0(new_n284));
  oaih12aa1n02x5               g189(.a(new_n284), .b(new_n279), .c(new_n280), .o1(new_n285));
  aob012aa1n03x5               g190(.a(new_n274), .b(\b[27] ), .c(\a[28] ), .out0(new_n286));
  oa0012aa1n12x5               g191(.a(new_n286), .b(\b[27] ), .c(\a[28] ), .o(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  nona22aa1n02x5               g194(.a(new_n285), .b(new_n288), .c(new_n289), .out0(new_n290));
  aoai13aa1n03x5               g195(.a(new_n289), .b(new_n288), .c(new_n272), .d(new_n284), .o1(new_n291));
  nanp02aa1n03x5               g196(.a(new_n291), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n275), .b(new_n289), .c(new_n276), .out0(new_n294));
  oaoi03aa1n09x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .o1(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n295), .c(new_n272), .d(new_n294), .o1(new_n297));
  oaih12aa1n02x5               g202(.a(new_n294), .b(new_n279), .c(new_n280), .o1(new_n298));
  nona22aa1n02x5               g203(.a(new_n298), .b(new_n295), .c(new_n296), .out0(new_n299));
  nanp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[30] ));
  nanb02aa1n02x5               g205(.a(new_n296), .b(new_n295), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n301), .b(\b[29] ), .c(\a[30] ), .o1(new_n302));
  norb02aa1n02x5               g207(.a(new_n294), .b(new_n296), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n303), .b(new_n279), .c(new_n280), .o1(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nona22aa1n02x5               g210(.a(new_n304), .b(new_n305), .c(new_n302), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n305), .b(new_n302), .c(new_n272), .d(new_n303), .o1(new_n307));
  nanp02aa1n03x5               g212(.a(new_n307), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n182), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g217(.a(new_n106), .b(new_n182), .c(new_n123), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g219(.a(new_n107), .b(new_n182), .c(new_n124), .out0(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g221(.a(new_n101), .b(new_n315), .c(new_n102), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g223(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


